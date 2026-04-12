#import "NimbleBridge.h"
#import "NimbleBridge+Internal.h"

#include <vector>
#include <string>
#include <memory>
#include <set>

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
    NSArray<NSNumber *> *_leftFootForce;
    NSArray<NSNumber *> *_rightFootForce;
    NSArray<NSNumber *> *_leftFootCoP;
    NSArray<NSNumber *> *_rightFootCoP;
    BOOL _leftFootInContact;
    BOOL _rightFootInContact;
    double _rootResidualNorm;
}

- (instancetype)initWithTorques:(NSArray<NSNumber *> *)torques {
    self = [super init];
    if (self) {
        _jointTorques = [torques copy];
        NSArray<NSNumber *> *zero3 = @[@0.0, @0.0, @0.0];
        _leftFootForce = zero3; _rightFootForce = zero3;
        _leftFootCoP = zero3; _rightFootCoP = zero3;
        _leftFootInContact = NO; _rightFootInContact = NO;
        _rootResidualNorm = 0;
    }
    return self;
}

- (instancetype)initWithTorques:(NSArray<NSNumber *> *)torques
                   leftForce:(NSArray<NSNumber *> *)leftForce
                   rightForce:(NSArray<NSNumber *> *)rightForce
                     leftCoP:(NSArray<NSNumber *> *)leftCoP
                    rightCoP:(NSArray<NSNumber *> *)rightCoP
               leftInContact:(BOOL)leftInContact
              rightInContact:(BOOL)rightInContact
            rootResidualNorm:(double)residual {
    self = [super init];
    if (self) {
        _jointTorques = [torques copy];
        _leftFootForce = [leftForce copy];
        _rightFootForce = [rightForce copy];
        _leftFootCoP = [leftCoP copy];
        _rightFootCoP = [rightCoP copy];
        _leftFootInContact = leftInContact;
        _rightFootInContact = rightInContact;
        _rootResidualNorm = residual;
    }
    return self;
}

- (NSArray<NSNumber *> *)jointTorques { return _jointTorques; }
- (NSArray<NSNumber *> *)leftFootForce { return _leftFootForce; }
- (NSArray<NSNumber *> *)rightFootForce { return _rightFootForce; }
- (NSArray<NSNumber *> *)leftFootCoP { return _leftFootCoP; }
- (NSArray<NSNumber *> *)rightFootCoP { return _rightFootCoP; }
- (BOOL)leftFootInContact { return _leftFootInContact; }
- (BOOL)rightFootInContact { return _rightFootInContact; }
- (double)rootResidualNorm { return _rootResidualNorm; }

@end

// MARK: - NimbleBridge

@implementation NimbleBridge {
    std::shared_ptr<dynamics::Skeleton> _skeleton;
    std::map<std::string, std::pair<dynamics::BodyNode*, Eigen::Vector3s>> _markers;
    BOOL _modelLoaded;

    // Ground-plane calibration for GRF estimation.
    // Held in the ARKit world frame (y-up). Auto-calibrated from a running
    // minimum of the subject's feet y-coordinates unless explicitly set.
    double _groundHeightY;
    BOOL _groundHeightCalibrated;
}

- (instancetype)init {
    self = [super init];
    if (self) {
        _modelLoaded = NO;
        _groundHeightY = 0.0;
        _groundHeightCalibrated = NO;
    }
    return self;
}

- (void)setGroundHeightY:(double)y {
    _groundHeightY = y;
    _groundHeightCalibrated = YES;
    NSLog(@"NimbleBridge: Ground plane set to y=%.4f m", y);
}

- (double)groundHeightY { return _groundHeightY; }
- (BOOL)groundHeightCalibrated { return _groundHeightCalibrated; }

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
        // Strategy is adaptive across the two supported models:
        //
        // - Rajagopal2016 (old default): single `torso` body from pelvis to
        //   shoulders, no separate spine/head/clavicle/scapula. ARKit spine
        //   / C7 / neck / head markers attach to the torso body with
        //   heuristic +Y offsets.
        //
        // - cyclistFullBodyMuscle.osim (new full-body): detailed spine with
        //   lumbar1-5, thoracic1-12, sacrum, head_neck, plus clavicle and
        //   scapula segments. We map ARKit markers to the closest real
        //   segment and let IK fit the multi-segment spine properly.
        //
        // A marker whose target body doesn't exist in the loaded skeleton is
        // silently skipped (and reported in the "missing bodies" log below),
        // so the same table works for both models.
        //
        // Each entry: ARKit marker name -> (body name, local offset in meters).
        struct VirtualMarker {
            const char* name;
            const char* bodyName;
            double offsetX, offsetY, offsetZ;
        };
        VirtualMarker virtualMarkers[] = {
            // Pelvis / lower extremities — all at body origin (= joint center)
            {"PELVIS",  "pelvis",    0.0,  0.0,  0.0},
            {"LHJC",    "femur_l",   0.0,  0.0,  0.0},
            {"RHJC",    "femur_r",   0.0,  0.0,  0.0},
            {"LKJC",    "tibia_l",   0.0,  0.0,  0.0},
            {"RKJC",    "tibia_r",   0.0,  0.0,  0.0},
            {"LAJC",    "talus_l",   0.0,  0.0,  0.0},
            {"RAJC",    "talus_r",   0.0,  0.0,  0.0},
            {"LTOE",    "toes_l",    0.0,  0.0,  0.0},
            {"RTOE",    "toes_r",    0.0,  0.0,  0.0},

            // --- Spine markers, new-model mapping (cyclist full body) ---
            // lumbar3 ≈ mid-lumbar, thoracic7 ≈ mid-thoracic, thoracic1 ≈ C7,
            // head_neck is a single body that spans neck + skull.
            {"SPINE_L", "lumbar3",   0.0,  0.0,  0.0},
            {"SPINE_M", "thoracic7", 0.0,  0.0,  0.0},
            {"C7",      "thoracic1", 0.0,  0.0,  0.0},
            {"NECK",    "head_neck", 0.0,  0.0,  0.0},
            {"HEAD",    "head_neck", 0.0,  0.15, 0.0},
            // --- Spine markers, old-model fallback (Rajagopal2016 torso) ---
            // Name-collision: for entries with the same marker name, only the
            // first-resolved body wins because `_markers[name] = ...` is a map
            // assignment. Since Rajagopal2016 doesn't have lumbar3 / thoracic*,
            // the cyclist-model rows above are silently skipped and these
            // torso-based fallbacks are used instead.
            {"SPINE_L", "torso",     0.0,  0.10, 0.0},
            {"SPINE_M", "torso",     0.0,  0.22, 0.0},
            {"C7",      "torso",     0.0,  0.38, 0.0},
            {"NECK",    "torso",     0.0,  0.44, 0.0},
            {"HEAD",    "torso",     0.0,  0.58, 0.0},

            // --- Upper extremities ---
            // New full-body model has clavicle + scapula bodies ahead of
            // humerus. ARKit's shoulder marker is the gleno-humeral joint
            // center, which corresponds to the humerus body origin on both
            // models, so the mapping is the same.
            {"LSJC",    "humerus_l", 0.0, 0.0,  0.0},
            {"RSJC",    "humerus_r", 0.0, 0.0,  0.0},
            {"LEJC",    "ulna_l",    0.0, 0.0,  0.0},
            {"REJC",    "ulna_r",    0.0, 0.0,  0.0},
            {"LWJC",    "hand_l",    0.0, 0.0,  0.0},
            {"RWJC",    "hand_r",    0.0, 0.0,  0.0},
        };

        // First-write-wins: cyclist-model rows come before Rajagopal2016
        // fallbacks in `virtualMarkers`, so on cyclist the detailed spine
        // mapping is installed first and any later fallback row for the
        // same marker is ignored. On Rajagopal2016 the cyclist rows don't
        // resolve (no lumbar3 / thoracic7 / head_neck bodies), so the
        // torso-based fallbacks fill in.
        std::set<std::string> attemptedMarkers;
        for (const auto& vm : virtualMarkers) {
            std::string markerName(vm.name);
            attemptedMarkers.insert(markerName);
            if (_markers.find(markerName) != _markers.end()) {
                continue;  // Already placed — earlier row won.
            }
            dynamics::BodyNode* body = _skeleton->getBodyNode(std::string(vm.bodyName));
            if (body) {
                _markers[markerName] = {
                    body,
                    Eigen::Vector3s(vm.offsetX, vm.offsetY, vm.offsetZ)
                };
            }
        }

        // Report: how many unique ARKit markers got placed vs. attempted.
        NSMutableArray<NSString *> *unplacedNames = [NSMutableArray array];
        for (const auto& name : attemptedMarkers) {
            if (_markers.find(name) == _markers.end()) {
                [unplacedNames addObject:[NSString stringWithUTF8String:name.c_str()]];
            }
        }
        NSLog(@"NimbleBridge: Placed %lu/%lu unique virtual markers",
              (unsigned long)(attemptedMarkers.size() - unplacedNames.count),
              (unsigned long)attemptedMarkers.size());
        if (unplacedNames.count > 0) {
            NSLog(@"NimbleBridge: ⚠ Unresolvable markers (no fallback body worked): %@",
                  [unplacedNames componentsJoinedByString:@", "]);
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

#pragma mark - NimbleBridge (Internal)

- (std::shared_ptr<dart::dynamics::Skeleton>)sharedSkeleton {
    // Returns the live shared_ptr. If no model is loaded this is a null
    // shared_ptr, which is an explicit "no skeleton yet" signal to callers
    // like MomentArmComputer. Any mutation through this pointer (positions,
    // body scales, external forces) is shared with every other holder —
    // intentional, so scaling and IK state stay consistent across objects.
    return _skeleton;
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
        // Upper extremity bodies (incl. clavicle + scapula on the new
        // full-body model, which exist in the kinematic chain between
        // torso/thoracic and humerus).
        if (bodyName.find("humerus") != std::string::npos ||
            bodyName.find("radius") != std::string::npos ||
            bodyName.find("ulna") != std::string::npos ||
            bodyName.find("hand") != std::string::npos ||
            bodyName.find("clavicle") != std::string::npos ||
            bodyName.find("scapula") != std::string::npos) {
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
        // Trunk: pelvis, torso, plus the detailed spine/ribcage on the new
        // full-body model — lumbar*, thoracic*, sacrum, rib*, sternum,
        // head_neck, Abdomen, Abd_*. All get the trunk scale.
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

// MARK: - ID with ground reaction force estimation
//
// This is the production ID path — it uses Nimble's built-in multi-contact,
// near-CoP inverse-dynamics solver to decompose the system wrench into
// per-foot GRFs plus joint torques. Foot contact is detected from the y
// coordinate of `calcn_l` / `calcn_r` versus a running ground-height
// estimate. When neither foot is in contact (flight), we fall back to
// plain getInverseDynamics which implicitly assumes zero external force.

static inline NSArray<NSNumber *> *vec3ToNSArray(const Eigen::Vector3s& v) {
    return @[@(v.x()), @(v.y()), @(v.z())];
}

- (nullable NimbleIDResult *)solveIDGRFWithJointAngles:(NSArray<NSNumber *> *)jointAngles
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
            q(i)   = [jointAngles[i] doubleValue];
            dq(i)  = [jointVelocities[i] doubleValue];
            ddq(i) = [jointAccelerations[i] doubleValue];
        }

        _skeleton->setPositions(q);
        _skeleton->setVelocities(dq);

        // --- 1. Ground height auto-calibration ---
        // Track the lowest observed calcn y across the session as a running
        // minimum. This is a cheap, monotonic estimate that doesn't need
        // explicit calibration. The offset (1 cm below the lowest observed
        // heel) absorbs the fact that calcn body origin sits slightly above
        // the true ground contact point.
        dynamics::BodyNode* calcnL = _skeleton->getBodyNode("calcn_l");
        dynamics::BodyNode* calcnR = _skeleton->getBodyNode("calcn_r");
        if (!calcnL || !calcnR) {
            // No foot bodies → can't do GRF; fall back to regular ID.
            Eigen::VectorXs torques = _skeleton->getInverseDynamics(ddq);
            NSMutableArray<NSNumber *> *ta = [NSMutableArray arrayWithCapacity:numDOFs];
            for (NSInteger i = 0; i < numDOFs; i++) [ta addObject:@(torques(i))];
            return [[NimbleIDResult alloc] initWithTorques:ta];
        }

        double calcnLY = calcnL->getWorldTransform().translation().y();
        double calcnRY = calcnR->getWorldTransform().translation().y();
        double lowest = std::min(calcnLY, calcnRY);

        if (!_groundHeightCalibrated) {
            _groundHeightY = lowest - 0.01;
            _groundHeightCalibrated = YES;
        } else if (lowest - 0.01 < _groundHeightY) {
            // Ratchet down if a lower foot position is observed.
            _groundHeightY = lowest - 0.01;
        }

        // --- 2. Contact detection ---
        // A foot is in contact if its heel y sits within CONTACT_THRESHOLD
        // of the ground and it's moving slowly (we gate on velocity too via
        // getCOMLinearVelocity later if needed — for baseline we go by
        // position alone and let the near-CoP solver absorb the rest).
        const double CONTACT_THRESHOLD = 0.06;  // 6 cm
        BOOL leftContact  = (calcnLY - _groundHeightY) < CONTACT_THRESHOLD;
        BOOL rightContact = (calcnRY - _groundHeightY) < CONTACT_THRESHOLD;

        // --- 3. Build initial wrench guesses (Newton-Euler baseline) ---
        // Whole-body weight: GRF_total ≈ mass * g (static stance). We split
        // 50/50 if both in contact, 100% on the single foot if only one,
        // and zero if airborne (caller falls through to plain ID).
        std::vector<const dynamics::BodyNode*> contactBodies;
        std::vector<Eigen::Vector6s> wrenchGuesses;

        double mass = _skeleton->getMass();
        Eigen::Vector3s weightUp(0.0, mass * 9.81, 0.0);

        int contactCount = (leftContact ? 1 : 0) + (rightContact ? 1 : 0);
        if (contactCount == 0) {
            // Flight phase — zero GRF, plain ID.
            Eigen::VectorXs torques = _skeleton->getInverseDynamics(ddq);
            NSMutableArray<NSNumber *> *ta = [NSMutableArray arrayWithCapacity:numDOFs];
            for (NSInteger i = 0; i < numDOFs; i++) [ta addObject:@(torques(i))];
            NSArray<NSNumber *> *zero3 = @[@0.0, @0.0, @0.0];
            return [[NimbleIDResult alloc] initWithTorques:ta
                                                 leftForce:zero3
                                                rightForce:zero3
                                                   leftCoP:zero3
                                                  rightCoP:zero3
                                             leftInContact:NO
                                            rightInContact:NO
                                          rootResidualNorm:0.0];
        }

        Eigen::Vector3s perFootForce = weightUp / contactCount;
        auto makeWrenchGuess = [&](dynamics::BodyNode* foot) -> Eigen::Vector6s {
            Eigen::Vector6s w;
            w.head<3>() = Eigen::Vector3s::Zero();   // torque (angular)
            w.tail<3>() = perFootForce;              // force  (linear, world)
            return w;
        };

        if (leftContact)  {
            contactBodies.push_back(calcnL);
            wrenchGuesses.push_back(makeWrenchGuess(calcnL));
        }
        if (rightContact) {
            contactBodies.push_back(calcnR);
            wrenchGuesses.push_back(makeWrenchGuess(calcnR));
        }

        // --- 4. Solve ---
        // getMultipleContactInverseDynamicsNearCoP finds per-foot wrenches
        // that (a) satisfy the Newton-Euler acceleration constraint, (b) sit
        // as close as possible to the wrench guesses, and (c) have their
        // center-of-pressure inside each foot's support polygon / on the
        // ground plane. Vertical axis = 1 (y).
        auto result = _skeleton->getMultipleContactInverseDynamicsNearCoP(
            ddq,
            contactBodies,
            wrenchGuesses,
            (s_t)_groundHeightY,
            1,          // vertical axis = y
            0.001,      // default weightForceToMeters
            false);     // not verbose

        // --- 5. Package results ---
        NSMutableArray<NSNumber *> *torqueArray = [NSMutableArray arrayWithCapacity:numDOFs];
        for (NSInteger i = 0; i < result.jointTorques.size(); i++) {
            [torqueArray addObject:@(result.jointTorques(i))];
        }

        // Pull left/right wrenches back out of result.contactWrenches in the
        // same order we pushed them. Extract force and CoP where applicable.
        Eigen::Vector3s zero3 = Eigen::Vector3s::Zero();
        Eigen::Vector3s leftForce = zero3, rightForce = zero3;
        Eigen::Vector3s leftCoP = zero3, rightCoP = zero3;
        size_t idx = 0;
        auto copFromWrench = [&](const Eigen::Vector6s& w, const Eigen::Vector3s& ctr)
            -> Eigen::Vector3s {
            // CoP on the ground plane: project from body origin along force
            // direction to hit y = _groundHeightY. For a vertical-y
            // convention, CoP_xz = body_xz + (body_y - ground_y) * f_xz/f_y
            Eigen::Vector3s force = w.tail<3>();
            if (std::abs(force.y()) < 1e-6) return ctr;
            double lambda = (ctr.y() - _groundHeightY) / force.y();
            Eigen::Vector3s cop = ctr - lambda * force;
            cop.y() = _groundHeightY;
            return cop;
        };
        if (leftContact) {
            leftForce = result.contactWrenches[idx].tail<3>();
            leftCoP = copFromWrench(result.contactWrenches[idx],
                                    calcnL->getWorldTransform().translation());
            idx++;
        }
        if (rightContact) {
            rightForce = result.contactWrenches[idx].tail<3>();
            rightCoP = copFromWrench(result.contactWrenches[idx],
                                     calcnR->getWorldTransform().translation());
            idx++;
        }

        // Root residual: the 6DoF component of jointTorques at the floating
        // root joint. For a correctly-modelled GRF system this should be
        // ~zero (the free joint has no actuator, so any residual there is
        // unphysical "error"). We take the norm as a scalar diagnostic.
        double rootResidual = 0.0;
        if (result.jointTorques.size() >= 6) {
            rootResidual = result.jointTorques.head<6>().norm();
        }

        return [[NimbleIDResult alloc] initWithTorques:torqueArray
                                             leftForce:vec3ToNSArray(leftForce)
                                            rightForce:vec3ToNSArray(rightForce)
                                               leftCoP:vec3ToNSArray(leftCoP)
                                              rightCoP:vec3ToNSArray(rightCoP)
                                         leftInContact:leftContact
                                        rightInContact:rightContact
                                      rootResidualNorm:rootResidual];
    } catch (const std::exception& e) {
        NSLog(@"NimbleBridge: ID+GRF exception: %s", e.what());
        return nil;
    } catch (...) {
        NSLog(@"NimbleBridge: ID+GRF unknown exception");
        return nil;
    }
}

@end
