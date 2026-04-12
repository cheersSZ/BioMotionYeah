#import "MomentArmComputer.h"

#include <vector>
#include <string>
#include <map>
#include <cmath>

#include <tinyxml2.h>

#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/biomechanics/OpenSimParser.hpp"
#include "dart/math/MathTypes.hpp"

using namespace dart;

// MARK: - Internal muscle path structure

struct InternalPathPoint {
    std::string bodyName;
    Eigen::Vector3s localOffset;
};

struct InternalMusclePath {
    std::string name;
    std::vector<InternalPathPoint> points;
    double maxIsometricForce;
    double optimalFiberLength;
    double tendonSlackLength;
    double pennationAngle;
};

// MARK: - MusclePathPoint

@implementation MusclePathPoint {
    NSString *_bodyName;
    double _x, _y, _z;
}

- (instancetype)initWithBody:(NSString *)body x:(double)x y:(double)y z:(double)z {
    self = [super init];
    if (self) { _bodyName = body; _x = x; _y = y; _z = z; }
    return self;
}
- (NSString *)bodyName { return _bodyName; }
- (double)x { return _x; }
- (double)y { return _y; }
- (double)z { return _z; }
@end

// MARK: - MusclePathData

@implementation MusclePathData {
    NSString *_name;
    NSArray<MusclePathPoint *> *_pathPoints;
    double _maxIsometricForce, _optimalFiberLength, _pennationAngle;
}

- (instancetype)initWithName:(NSString *)name
                  pathPoints:(NSArray<MusclePathPoint *> *)points
           maxIsometricForce:(double)f0
        optimalFiberLength:(double)l0
             pennationAngle:(double)alpha {
    self = [super init];
    if (self) {
        _name = name; _pathPoints = points;
        _maxIsometricForce = f0; _optimalFiberLength = l0; _pennationAngle = alpha;
    }
    return self;
}
- (NSString *)name { return _name; }
- (NSArray<MusclePathPoint *> *)pathPoints { return _pathPoints; }
- (double)maxIsometricForce { return _maxIsometricForce; }
- (double)optimalFiberLength { return _optimalFiberLength; }
- (double)pennationAngle { return _pennationAngle; }
@end

// MARK: - MomentArmComputer

@implementation MomentArmComputer {
    std::vector<InternalMusclePath> _musclePaths;
    std::shared_ptr<dynamics::Skeleton> _skeleton;
    BOOL _loaded;
}

- (instancetype)init {
    self = [super init];
    if (self) { _loaded = NO; }
    return self;
}

- (BOOL)parseMusclePathsFromOsimPath:(NSString *)path {
    std::string pathStr([path UTF8String]);

    // Load skeleton via Nimble
    auto osimFile = biomechanics::OpenSimParser::parseOsim(pathStr);
    if (!osimFile.skeleton) return NO;
    _skeleton = osimFile.skeleton;

    // Parse muscle paths from XML directly (Nimble doesn't parse muscles)
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(pathStr.c_str()) != tinyxml2::XML_SUCCESS) return NO;

    auto* root = doc.FirstChildElement("OpenSimDocument");
    if (!root) return NO;
    auto* model = root->FirstChildElement("Model");
    if (!model) return NO;
    auto* forceSet = model->FirstChildElement("ForceSet");
    if (!forceSet) return NO;
    auto* objects = forceSet->FirstChildElement("objects");
    if (!objects) return NO;

    _musclePaths.clear();

    for (auto* muscleEl = objects->FirstChildElement("Millard2012EquilibriumMuscle");
         muscleEl;
         muscleEl = muscleEl->NextSiblingElement("Millard2012EquilibriumMuscle")) {

        InternalMusclePath mp;
        const char* name = muscleEl->Attribute("name");
        if (!name) continue;
        mp.name = name;

        // Read muscle params
        auto readDouble = [&](const char* tag) -> double {
            auto* el = muscleEl->FirstChildElement(tag);
            return (el && el->GetText()) ? std::stod(el->GetText()) : 0.0;
        };

        mp.maxIsometricForce = readDouble("max_isometric_force");
        mp.optimalFiberLength = readDouble("optimal_fiber_length");
        mp.tendonSlackLength = readDouble("tendon_slack_length");
        mp.pennationAngle = readDouble("pennation_angle_at_optimal");

        // Parse GeometryPath → PathPointSet → PathPoint
        auto* geoPath = muscleEl->FirstChildElement("GeometryPath");
        if (!geoPath) continue;
        auto* pathPointSet = geoPath->FirstChildElement("PathPointSet");
        if (!pathPointSet) continue;
        auto* ppObjects = pathPointSet->FirstChildElement("objects");
        if (!ppObjects) continue;

        for (auto* pp = ppObjects->FirstChildElement("PathPoint");
             pp; pp = pp->NextSiblingElement("PathPoint")) {

            InternalPathPoint ipp;

            // Get body name
            auto* bodyEl = pp->FirstChildElement("body");
            // Newer format uses socket_parent_frame
            if (!bodyEl) bodyEl = pp->FirstChildElement("socket_parent_frame");
            if (bodyEl && bodyEl->GetText()) {
                std::string bodyRef = bodyEl->GetText();
                // Strip path prefix if present (e.g., "/bodyset/tibia_r" → "tibia_r")
                auto lastSlash = bodyRef.find_last_of('/');
                if (lastSlash != std::string::npos) {
                    bodyRef = bodyRef.substr(lastSlash + 1);
                }
                ipp.bodyName = bodyRef;
            } else {
                continue;
            }

            // Get location
            auto* locEl = pp->FirstChildElement("location");
            if (locEl && locEl->GetText()) {
                std::string locStr = locEl->GetText();
                std::istringstream ss(locStr);
                double x = 0, y = 0, z = 0;
                ss >> x >> y >> z;
                ipp.localOffset = Eigen::Vector3s(x, y, z);
            } else {
                ipp.localOffset = Eigen::Vector3s::Zero();
            }

            mp.points.push_back(ipp);
        }

        if (mp.points.size() >= 2 && mp.maxIsometricForce > 0) {
            _musclePaths.push_back(mp);
        }
    }

    _loaded = !_musclePaths.empty();
    NSLog(@"MomentArmComputer: Parsed %lu muscle paths", (unsigned long)_musclePaths.size());
    return _loaded;
}

- (NSInteger)numMuscles {
    return (NSInteger)_musclePaths.size();
}

- (NSArray<NSString *> *)muscleNames {
    NSMutableArray *names = [NSMutableArray array];
    for (const auto& mp : _musclePaths) {
        [names addObject:[NSString stringWithUTF8String:mp.name.c_str()]];
    }
    return names;
}

- (nullable MusclePathData *)musclePathDataForName:(NSString *)name {
    std::string nameStr([name UTF8String]);
    for (const auto& mp : _musclePaths) {
        if (mp.name == nameStr) {
            NSMutableArray *points = [NSMutableArray array];
            for (const auto& pp : mp.points) {
                [points addObject:[[MusclePathPoint alloc]
                    initWithBody:[NSString stringWithUTF8String:pp.bodyName.c_str()]
                    x:pp.localOffset.x() y:pp.localOffset.y() z:pp.localOffset.z()]];
            }
            return [[MusclePathData alloc] initWithName:name
                                             pathPoints:points
                                      maxIsometricForce:mp.maxIsometricForce
                                   optimalFiberLength:mp.optimalFiberLength
                                        pennationAngle:mp.pennationAngle];
        }
    }
    return nil;
}

/// Compute total musculotendon path length for a muscle at the current skeleton configuration.
- (double)computeMuscleLengthForIndex:(NSInteger)muscleIdx {
    if (!_skeleton || muscleIdx < 0 || muscleIdx >= (NSInteger)_musclePaths.size()) return 0;

    const auto& mp = _musclePaths[muscleIdx];
    double totalLength = 0;

    for (size_t i = 1; i < mp.points.size(); i++) {
        Eigen::Vector3s worldA = [self worldPositionForPathPoint:mp.points[i-1]];
        Eigen::Vector3s worldB = [self worldPositionForPathPoint:mp.points[i]];
        totalLength += (worldB - worldA).norm();
    }

    return totalLength;
}

/// Transform a path point from body-local coordinates to world coordinates.
- (Eigen::Vector3s)worldPositionForPathPoint:(const InternalPathPoint&)pp {
    dynamics::BodyNode* body = _skeleton->getBodyNode(pp.bodyName);
    if (!body) return pp.localOffset;  // fallback

    Eigen::Isometry3s worldTransform = body->getWorldTransform();
    return worldTransform * pp.localOffset;
}

- (nullable NSArray<NSNumber *> *)computeMomentArmsWithJointAngles:(NSArray<NSNumber *> *)jointAngles
                                                          dofNames:(NSArray<NSString *> *)dofNames {
    if (!_loaded || !_skeleton) return nil;

    NSInteger nMuscles = (NSInteger)_musclePaths.size();
    NSInteger nDOFs = dofNames.count;

    // Set skeleton to the given configuration
    Eigen::VectorXs q = _skeleton->getPositions();
    std::map<std::string, int> dofToSkeletonIdx;

    for (size_t i = 0; i < _skeleton->getNumDofs(); i++) {
        dofToSkeletonIdx[_skeleton->getDof(i)->getName()] = (int)i;
    }

    // Set the joint angles we have
    for (NSInteger i = 0; i < nDOFs; i++) {
        std::string dofName([dofNames[i] UTF8String]);
        auto it = dofToSkeletonIdx.find(dofName);
        if (it != dofToSkeletonIdx.end()) {
            q(it->second) = [jointAngles[i] doubleValue];
        }
    }
    _skeleton->setPositions(q);

    // Compute baseline muscle lengths
    std::vector<double> baselineLengths(nMuscles);
    for (NSInteger m = 0; m < nMuscles; m++) {
        baselineLengths[m] = [self computeMuscleLengthForIndex:m];
    }

    // Numerical differentiation: r_ij = -(L_i(q + eps*e_j) - L_i(q - eps*e_j)) / (2*eps)
    const double eps = 1e-4;  // radians
    NSMutableArray<NSNumber *> *momentArms = [NSMutableArray arrayWithCapacity:nMuscles * nDOFs];

    // Initialize with zeros
    for (NSInteger i = 0; i < nMuscles * nDOFs; i++) {
        [momentArms addObject:@(0.0)];
    }

    for (NSInteger j = 0; j < nDOFs; j++) {
        std::string dofName([dofNames[j] UTF8String]);
        auto it = dofToSkeletonIdx.find(dofName);
        if (it == dofToSkeletonIdx.end()) continue;
        int skelIdx = it->second;

        // Perturb +eps
        Eigen::VectorXs qPlus = q;
        qPlus(skelIdx) += eps;
        _skeleton->setPositions(qPlus);

        std::vector<double> lengthsPlus(nMuscles);
        for (NSInteger m = 0; m < nMuscles; m++) {
            lengthsPlus[m] = [self computeMuscleLengthForIndex:m];
        }

        // Perturb -eps
        Eigen::VectorXs qMinus = q;
        qMinus(skelIdx) -= eps;
        _skeleton->setPositions(qMinus);

        std::vector<double> lengthsMinus(nMuscles);
        for (NSInteger m = 0; m < nMuscles; m++) {
            lengthsMinus[m] = [self computeMuscleLengthForIndex:m];
        }

        // Compute moment arms: r = -dL/dq
        for (NSInteger m = 0; m < nMuscles; m++) {
            double r = -(lengthsPlus[m] - lengthsMinus[m]) / (2.0 * eps);
            momentArms[m * nDOFs + j] = @(r);
        }
    }

    // Restore original configuration
    _skeleton->setPositions(q);

    return momentArms;
}

- (NSArray<NSNumber *> *)currentMuscleLengths {
    if (!_loaded || !_skeleton) return @[];
    NSMutableArray<NSNumber *> *lengths =
        [NSMutableArray arrayWithCapacity:_musclePaths.size()];
    for (NSInteger i = 0; i < (NSInteger)_musclePaths.size(); i++) {
        [lengths addObject:@([self computeMuscleLengthForIndex:i])];
    }
    return lengths;
}

- (NSArray<NSNumber *> *)maxIsometricForces {
    NSMutableArray<NSNumber *> *forces =
        [NSMutableArray arrayWithCapacity:_musclePaths.size()];
    for (const auto& mp : _musclePaths) [forces addObject:@(mp.maxIsometricForce)];
    return forces;
}

- (NSArray<NSNumber *> *)optimalFiberLengths {
    NSMutableArray<NSNumber *> *lens =
        [NSMutableArray arrayWithCapacity:_musclePaths.size()];
    for (const auto& mp : _musclePaths) [lens addObject:@(mp.optimalFiberLength)];
    return lens;
}

- (NSArray<NSNumber *> *)tendonSlackLengths {
    NSMutableArray<NSNumber *> *lens =
        [NSMutableArray arrayWithCapacity:_musclePaths.size()];
    for (const auto& mp : _musclePaths) [lens addObject:@(mp.tendonSlackLength)];
    return lens;
}

- (NSArray<NSNumber *> *)pennationAngles {
    NSMutableArray<NSNumber *> *angs =
        [NSMutableArray arrayWithCapacity:_musclePaths.size()];
    for (const auto& mp : _musclePaths) [angs addObject:@(mp.pennationAngle)];
    return angs;
}

@end
