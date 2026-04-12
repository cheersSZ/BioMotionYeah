#import "NimbleBridge.h"

#include <vector>
#include <string>
#include <memory>

#include "dart/biomechanics/OpenSimParser.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/math/MathTypes.hpp"

using namespace dart;

// MARK: - NimbleIKResult

@implementation NimbleIKResult {
    NSArray<NSNumber *> *_jointAngles;
    double _error;
    NSInteger _numDOFs;
    NSArray<NSString *> *_dofNames;
}

- (instancetype)initWithAngles:(NSArray<NSNumber *> *)angles
                         error:(double)error
                       numDOFs:(NSInteger)numDOFs
                      dofNames:(NSArray<NSString *> *)dofNames {
    self = [super init];
    if (self) {
        _jointAngles = [angles copy];
        _error = error;
        _numDOFs = numDOFs;
        _dofNames = [dofNames copy];
    }
    return self;
}

- (NSArray<NSNumber *> *)jointAngles { return _jointAngles; }
- (double)error { return _error; }
- (NSInteger)numDOFs { return _numDOFs; }
- (NSArray<NSString *> *)dofNames { return _dofNames; }

@end

// MARK: - NimbleIDResult

@implementation NimbleIDResult {
    NSArray<NSNumber *> *_jointTorques;
}

- (instancetype)initWithTorques:(NSArray<NSNumber *> *)torques {
    self = [super init];
    if (self) {
        _jointTorques = [torques copy];
    }
    return self;
}

- (NSArray<NSNumber *> *)jointTorques { return _jointTorques; }

@end

// MARK: - NimbleBridge

@implementation NimbleBridge {
    std::shared_ptr<dynamics::Skeleton> _skeleton;
    std::map<std::string, std::pair<dynamics::BodyNode*, Eigen::Vector3s>> _markers;
    BOOL _modelLoaded;
}

- (instancetype)init {
    self = [super init];
    if (self) {
        _modelLoaded = NO;
    }
    return self;
}

- (BOOL)loadModelFromPath:(NSString *)path {
    try {
        std::string pathStr = std::string([path UTF8String]);
        NSLog(@"NimbleBridge: Loading model from %@", path);

        // Parse the .osim file
        biomechanics::OpenSimFile osimFile = biomechanics::OpenSimParser::parseOsim(pathStr);

        if (!osimFile.skeleton) {
            NSLog(@"NimbleBridge: Failed to parse skeleton from %@", path);
            return NO;
        }

        _skeleton = osimFile.skeleton;

        // Store the model's own markers
        _markers.clear();
        for (const auto& [name, pair] : osimFile.markersMap) {
            _markers[name] = pair;
        }

        // Register virtual markers at joint centers for ARKit compatibility.
        // ARKit gives us joint-center positions, not surface-marker positions,
        // so we attach one marker per body at a well-defined local point.
        //
        // For Rajagopal2016 the spine is a single rigid `torso` body from pelvis
        // top to shoulders; there are no separate lumbar/thoracic/cervical/head
        // segments. We still want ARKit's spine/C7/neck/head markers to feed IK
        // (they help constrain torso orientation), so we attach them to the
        // torso body at heuristic vertical offsets that approximate where the
        // real landmarks sit along an average torso. MarkerFitter in M6 will
        // refine these offsets per subject.
        //
        // Each entry: ARKit marker name -> (body name, local offset in meters).
        struct VirtualMarker {
            const char* name;
            const char* bodyName;
            double offsetX, offsetY, offsetZ;
        };
        VirtualMarker virtualMarkers[] = {
            // Pelvis / lower extremities — all at body origin (= joint center)
            {"PELVIS",  "pelvis",   0.0,  0.0,  0.0},
            {"LHJC",    "femur_l",  0.0,  0.0,  0.0},
            {"RHJC",    "femur_r",  0.0,  0.0,  0.0},
            {"LKJC",    "tibia_l",  0.0,  0.0,  0.0},
            {"RKJC",    "tibia_r",  0.0,  0.0,  0.0},
            {"LAJC",    "talus_l",  0.0,  0.0,  0.0},
            {"RAJC",    "talus_r",  0.0,  0.0,  0.0},
            {"LTOE",    "toes_l",   0.0,  0.0,  0.0},
            {"RTOE",    "toes_r",   0.0,  0.0,  0.0},
            // Torso (single rigid segment in Rajagopal2016) — distribute markers
            // up the body's +Y axis (torso body Y points from base to top).
            {"SPINE_L", "torso",    0.0,  0.10, 0.0},
            {"SPINE_M", "torso",    0.0,  0.22, 0.0},
            {"C7",      "torso",    0.0,  0.38, 0.0},
            {"NECK",    "torso",    0.0,  0.44, 0.0},
            {"HEAD",    "torso",    0.0,  0.58, 0.0},
            // Upper extremities — joint centers at each body's origin.
            {"LSJC",    "humerus_l", 0.0, 0.0,  0.0},
            {"RSJC",    "humerus_r", 0.0, 0.0,  0.0},
            {"LEJC",    "ulna_l",    0.0, 0.0,  0.0},
            {"REJC",    "ulna_r",    0.0, 0.0,  0.0},
            {"LWJC",    "hand_l",    0.0, 0.0,  0.0},
            {"RWJC",    "hand_r",    0.0, 0.0,  0.0},
        };

        int addedVirtual = 0;
        NSMutableArray<NSString *> *missingBodies = [NSMutableArray array];
        for (const auto& vm : virtualMarkers) {
            dynamics::BodyNode* body = _skeleton->getBodyNode(std::string(vm.bodyName));
            if (body) {
                _markers[std::string(vm.name)] = {
                    body,
                    Eigen::Vector3s(vm.offsetX, vm.offsetY, vm.offsetZ)
                };
                addedVirtual++;
            } else {
                [missingBodies addObject:
                    [NSString stringWithFormat:@"%s(→%s)", vm.name, vm.bodyName]];
            }
        }
        NSLog(@"NimbleBridge: Added %d/%lu virtual markers at joint centers",
              addedVirtual, sizeof(virtualMarkers)/sizeof(virtualMarkers[0]));
        if (missingBodies.count > 0) {
            NSLog(@"NimbleBridge: Skipped markers with missing bodies: %@",
                  [missingBodies componentsJoinedByString:@", "]);
        }

        _modelLoaded = YES;
        NSLog(@"NimbleBridge: Loaded model with %ld DOFs, %lu markers",
              (long)_skeleton->getNumDofs(), (unsigned long)_markers.size());
        return YES;
    } catch (const std::exception& e) {
        NSLog(@"NimbleBridge: C++ exception loading model: %s", e.what());
        return NO;
    } catch (...) {
        NSLog(@"NimbleBridge: Unknown exception loading model");
        return NO;
    }
}

- (BOOL)isModelLoaded {
    return _modelLoaded;
}

- (double)totalMass {
    if (!_modelLoaded || !_skeleton) return 0.0;
    return _skeleton->getMass();
}

- (NSInteger)numDOFs {
    if (!_modelLoaded) return 0;
    return (NSInteger)_skeleton->getNumDofs();
}

- (NSArray<NSString *> *)dofNames {
    if (!_modelLoaded) return @[];
    NSMutableArray *names = [NSMutableArray array];
    for (size_t i = 0; i < _skeleton->getNumDofs(); i++) {
        std::string name = _skeleton->getDof(i)->getName();
        [names addObject:[NSString stringWithUTF8String:name.c_str()]];
    }
    return names;
}

- (NSArray<NSString *> *)markerNames {
    if (!_modelLoaded) return @[];
    NSMutableArray *names = [NSMutableArray array];
    for (const auto& [name, _] : _markers) {
        [names addObject:[NSString stringWithUTF8String:name.c_str()]];
    }
    return names;
}

- (BOOL)scaleModelWithHeight:(double)height
             markerPositions:(NSArray<NSNumber *> *)markerPositions
                 markerNames:(NSArray<NSString *> *)markerNames {
    if (!_modelLoaded) return NO;

    // Per-segment anthropometric scaling.
    //
    // Uniform height-ratio scaling (the previous implementation) is wrong:
    // real anthropometry varies between segments even for subjects of the
    // same height (e.g. limb length relative to height varies ±5%).
    //
    // If the IK pass has already been run and `markerPositions` contain
    // joint-center positions, we derive per-group scale factors directly
    // from inter-joint distances measured in the ARKit world:
    //
    //   lower-limb scale = (hip→ankle distance, averaged L/R) / model default
    //   trunk scale      = (pelvis→shoulder-midpoint distance) / model default
    //   upper-limb scale = (shoulder→wrist distance, averaged L/R) / model default
    //
    // If markers are missing, we fall back to height/1.8 for that group.
    //
    // For Rajagopal2016 with default male proportions the approximate
    // reference segment lengths at 1.8m standing height are:
    //   lower extremity (hip→ankle)    ≈ 0.88 m
    //   trunk (pelvis→shoulder midpt)  ≈ 0.52 m
    //   upper extremity (shoulder→wrist) ≈ 0.54 m

    auto markerWorld = [&](const std::string& name) -> Eigen::Vector3s {
        for (NSUInteger i = 0; i < markerNames.count; i++) {
            if (std::string([markerNames[i] UTF8String]) == name &&
                i * 3 + 2 < markerPositions.count) {
                return Eigen::Vector3s(
                    [markerPositions[i * 3 + 0] doubleValue],
                    [markerPositions[i * 3 + 1] doubleValue],
                    [markerPositions[i * 3 + 2] doubleValue]
                );
            }
        }
        return Eigen::Vector3s::Zero();
    };
    auto hasMarker = [&](const std::string& name) -> bool {
        for (NSUInteger i = 0; i < markerNames.count; i++) {
            if (std::string([markerNames[i] UTF8String]) == name) return true;
        }
        return false;
    };

    const double fallbackScale = height / 1.8;

    auto segLength = [&](const std::string& a, const std::string& b) -> double {
        if (!hasMarker(a) || !hasMarker(b)) return -1.0;
        return (markerWorld(a) - markerWorld(b)).norm();
    };

    // Lower extremity: average of L and R
    double lowerScale = fallbackScale;
    double lhl = segLength("LHJC", "LAJC");
    double rhl = segLength("RHJC", "RAJC");
    std::vector<double> lowerLengths;
    if (lhl > 0) lowerLengths.push_back(lhl);
    if (rhl > 0) lowerLengths.push_back(rhl);
    if (!lowerLengths.empty()) {
        double avg = 0;
        for (double l : lowerLengths) avg += l;
        avg /= lowerLengths.size();
        lowerScale = avg / 0.88;  // reference lower-limb length at 1.8m height
    }

    // Upper extremity: average of L and R
    double upperScale = fallbackScale;
    double lal = segLength("LSJC", "LWJC");
    double ral = segLength("RSJC", "RWJC");
    std::vector<double> upperLengths;
    if (lal > 0) upperLengths.push_back(lal);
    if (ral > 0) upperLengths.push_back(ral);
    if (!upperLengths.empty()) {
        double avg = 0;
        for (double l : upperLengths) avg += l;
        avg /= upperLengths.size();
        upperScale = avg / 0.54;  // reference upper-limb length at 1.8m height
    }

    // Trunk: pelvis to midpoint of shoulders
    double trunkScale = fallbackScale;
    if (hasMarker("PELVIS") && hasMarker("LSJC") && hasMarker("RSJC")) {
        Eigen::Vector3s p = markerWorld("PELVIS");
        Eigen::Vector3s shoulderMid = 0.5 * (markerWorld("LSJC") + markerWorld("RSJC"));
        double len = (shoulderMid - p).norm();
        if (len > 0.1) {
            trunkScale = len / 0.52;  // reference pelvis→shoulder length at 1.8m
        }
    }

    // Clamp to sensible anthropometric bounds so a bad single frame can't
    // blow up the skeleton (e.g. partially-tracked frame with wrist near ground).
    auto clampScale = [](double s) { return std::max(0.7, std::min(1.4, s)); };
    lowerScale = clampScale(lowerScale);
    upperScale = clampScale(upperScale);
    trunkScale = clampScale(trunkScale);

    // Assign per-body scale: nimble expects a flat VectorXs of size 3*numBodies
    // arranged as [x,y,z, x,y,z, ...]. We group each body into lower/trunk/upper.
    size_t numBodies = _skeleton->getNumBodyNodes();
    Eigen::VectorXs bodyScales(numBodies * 3);

    auto groupScale = [&](const std::string& bodyName) -> double {
        // Upper extremity bodies
        if (bodyName.find("humerus") != std::string::npos ||
            bodyName.find("radius") != std::string::npos ||
            bodyName.find("ulna") != std::string::npos ||
            bodyName.find("hand") != std::string::npos) {
            return upperScale;
        }
        // Lower extremity bodies
        if (bodyName.find("femur") != std::string::npos ||
            bodyName.find("tibia") != std::string::npos ||
            bodyName.find("patella") != std::string::npos ||
            bodyName.find("talus") != std::string::npos ||
            bodyName.find("calcn") != std::string::npos ||
            bodyName.find("toes") != std::string::npos) {
            return lowerScale;
        }
        // Torso, pelvis, head, neck → trunk
        return trunkScale;
    };

    for (size_t i = 0; i < numBodies; i++) {
        auto* body = _skeleton->getBodyNode(i);
        double s = groupScale(body->getName());
        bodyScales(i * 3 + 0) = s;
        bodyScales(i * 3 + 1) = s;
        bodyScales(i * 3 + 2) = s;
    }
    _skeleton->setBodyScales(bodyScales);

    NSLog(@"NimbleBridge: Per-segment scale — lower %.3f, trunk %.3f, upper %.3f (height fallback %.3f)",
          lowerScale, trunkScale, upperScale, fallbackScale);

    return YES;
}

- (nullable NimbleIKResult *)solveIKWithMarkerPositions:(NSArray<NSNumber *> *)markerPositions
                                            markerNames:(NSArray<NSString *> *)markerNames {
    if (!_modelLoaded) return nil;
    if (markerPositions.count != markerNames.count * 3) return nil;

    // Build marker list — ONLY include markers that exist in the model (no nullptrs)
    std::vector<std::pair<dynamics::BodyNode*, Eigen::Vector3s>> markerList;
    std::vector<Eigen::Vector3s> targetList;

    for (NSUInteger i = 0; i < markerNames.count; i++) {
        std::string name = std::string([markerNames[i] UTF8String]);

        auto it = _markers.find(name);
        if (it == _markers.end() || it->second.first == nullptr) {
            continue;  // Skip unknown markers entirely — don't pass nullptr to Nimble
        }

        markerList.push_back(it->second);
        targetList.push_back(Eigen::Vector3s(
            [markerPositions[i * 3 + 0] doubleValue],
            [markerPositions[i * 3 + 1] doubleValue],
            [markerPositions[i * 3 + 2] doubleValue]
        ));
    }

    if (markerList.empty()) return nil;

    // Flatten target positions and create uniform weights
    Eigen::VectorXs targetPositions(markerList.size() * 3);
    for (size_t i = 0; i < targetList.size(); i++) {
        targetPositions(i * 3 + 0) = targetList[i].x();
        targetPositions(i * 3 + 1) = targetList[i].y();
        targetPositions(i * 3 + 2) = targetList[i].z();
    }
    Eigen::VectorXs weights = Eigen::VectorXs::Ones(markerList.size());

    try {
        // Solve IK
        double error = _skeleton->fitMarkersToWorldPositions(
            markerList, targetPositions, weights, false);

        // Extract joint angles
        Eigen::VectorXs positions = _skeleton->getPositions();
        NSMutableArray<NSNumber *> *angles = [NSMutableArray arrayWithCapacity:positions.size()];
        for (int i = 0; i < positions.size(); i++) {
            [angles addObject:@(positions(i))];
        }

        return [[NimbleIKResult alloc] initWithAngles:angles
                                                error:error
                                              numDOFs:positions.size()
                                             dofNames:self.dofNames];
    } catch (const std::exception& e) {
        NSLog(@"NimbleBridge: IK exception: %s", e.what());
        return nil;
    } catch (...) {
        NSLog(@"NimbleBridge: IK unknown exception");
        return nil;
    }
}

- (nullable NimbleIDResult *)solveIDWithJointAngles:(NSArray<NSNumber *> *)jointAngles
                                   jointVelocities:(NSArray<NSNumber *> *)jointVelocities
                               jointAccelerations:(NSArray<NSNumber *> *)jointAccelerations {
    if (!_modelLoaded) return nil;

    NSInteger numDOFs = (NSInteger)_skeleton->getNumDofs();
    if (jointAngles.count != numDOFs ||
        jointVelocities.count != numDOFs ||
        jointAccelerations.count != numDOFs) {
        return nil;
    }

    try {
        // Set skeleton state
        Eigen::VectorXs q(numDOFs), dq(numDOFs), ddq(numDOFs);
        for (NSInteger i = 0; i < numDOFs; i++) {
            q(i) = [jointAngles[i] doubleValue];
            dq(i) = [jointVelocities[i] doubleValue];
            ddq(i) = [jointAccelerations[i] doubleValue];
        }

        _skeleton->setPositions(q);
        _skeleton->setVelocities(dq);

        // Compute inverse dynamics: tau = M*ddq + C(q,dq) - J^T * F_ext
        Eigen::VectorXs torques = _skeleton->getInverseDynamics(ddq);

        // Convert to NSArray
        NSMutableArray<NSNumber *> *torqueArray = [NSMutableArray arrayWithCapacity:numDOFs];
        for (NSInteger i = 0; i < numDOFs; i++) {
            [torqueArray addObject:@(torques(i))];
        }

        return [[NimbleIDResult alloc] initWithTorques:torqueArray];
    } catch (const std::exception& e) {
        NSLog(@"NimbleBridge: ID exception: %s", e.what());
        return nil;
    } catch (...) {
        NSLog(@"NimbleBridge: ID unknown exception");
        return nil;
    }
}

@end
