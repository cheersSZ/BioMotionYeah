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
        // ARKit gives us joint center positions, not surface marker positions.
        // Map our JointMapping names to the appropriate body nodes at their origins.
        struct VirtualMarker { const char* name; const char* bodyName; };
        VirtualMarker virtualMarkers[] = {
            {"PELVIS", "pelvis"},
            {"LHJC", "femur_l"},    {"RHJC", "femur_r"},
            {"LKJC", "tibia_l"},    {"RKJC", "tibia_r"},
            {"LAJC", "talus_l"},    {"RAJC", "talus_r"},
            {"LTOE", "toes_l"},     {"RTOE", "toes_r"},
            {"SPINE_L", "lumbar5"}, {"SPINE_M", "lumbar3"},
            {"C7", "thorax"},       {"NECK", "thorax"},
            {"HEAD", "head"},
            {"LSJC", "humerus_l"},  {"RSJC", "humerus_r"},
            {"LEJC", "ulna_l"},     {"REJC", "ulna_r"},
            {"LWJC", "hand_l"},     {"RWJC", "hand_r"},
        };

        int addedVirtual = 0;
        for (const auto& vm : virtualMarkers) {
            dynamics::BodyNode* body = _skeleton->getBodyNode(std::string(vm.bodyName));
            if (body) {
                _markers[std::string(vm.name)] = {body, Eigen::Vector3s::Zero()};
                addedVirtual++;
            }
        }
        NSLog(@"NimbleBridge: Added %d virtual markers at joint centers", addedVirtual);

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

    // Simple scaling: scale all body segments by height ratio
    // Default model height is approximately 1.8m
    double scaleFactor = height / 1.8;
    _skeleton->setBodyScales(Eigen::VectorXs::Constant(
        _skeleton->getNumBodyNodes() * 3, scaleFactor));

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
