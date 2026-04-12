#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

/// Parsed muscle path point (body name + local offset).
@interface MusclePathPoint : NSObject
@property (nonatomic, readonly) NSString *bodyName;
@property (nonatomic, readonly) double x, y, z;  // Local offset in body frame (meters)
@end

/// Parsed muscle with path geometry for moment arm computation.
@interface MusclePathData : NSObject
@property (nonatomic, readonly) NSString *name;
@property (nonatomic, readonly) NSArray<MusclePathPoint *> *pathPoints;
@property (nonatomic, readonly) double maxIsometricForce;
@property (nonatomic, readonly) double optimalFiberLength;
@property (nonatomic, readonly) double pennationAngle;
@end

/// Computes moment arms from muscle path geometry using numerical differentiation
/// of musculotendon length with respect to joint angles: r = -dL/dq.
///
/// Uses the Nimble skeleton's forward kinematics to transform muscle attachment
/// points to world coordinates and compute total muscle-tendon path length.
@interface MomentArmComputer : NSObject

/// Parse muscle path geometry from a .osim file.
/// @param path Path to the .osim file.
/// @return YES if muscles were parsed successfully.
- (BOOL)parseMusclePathsFromOsimPath:(NSString *)path;

/// Number of muscles with parsed paths.
@property (nonatomic, readonly) NSInteger numMuscles;

/// Compute the moment arm matrix R(q) at the current skeleton configuration.
/// Returns a flat row-major matrix [nMuscles x nDOFs] where R[i][j] = -dL_i/dq_j.
///
/// @param jointAngles Current joint angles (from IK).
/// @param dofNames DOF names corresponding to the angle array.
/// @return Flat array of moment arms (nMuscles * nDOFs), or nil on failure.
- (nullable NSArray<NSNumber *> *)computeMomentArmsWithJointAngles:(NSArray<NSNumber *> *)jointAngles
                                                          dofNames:(NSArray<NSString *> *)dofNames;

/// Current musculotendon path lengths L_MT(q) for all parsed muscles, at the
/// skeleton's CURRENT pose (as left by the most recent call to
/// computeMomentArmsWithJointAngles:dofNames:). Lengths are in meters.
/// Returns an array of size numMuscles.
@property (nonatomic, readonly) NSArray<NSNumber *> *currentMuscleLengths;

/// Maximum isometric force F_max for each parsed muscle, in newtons.
/// Array of size numMuscles, in the same order as currentMuscleLengths.
@property (nonatomic, readonly) NSArray<NSNumber *> *maxIsometricForces;

/// Optimal fiber length l_opt for each muscle, in meters.
@property (nonatomic, readonly) NSArray<NSNumber *> *optimalFiberLengths;

/// Tendon slack length l_Ts for each muscle, in meters. Falls back to
/// 0.0 if the underlying path did not parse this field.
@property (nonatomic, readonly) NSArray<NSNumber *> *tendonSlackLengths;

/// Pennation angle α₀ at optimal fiber length, in radians.
@property (nonatomic, readonly) NSArray<NSNumber *> *pennationAngles;

/// Get the muscle path data for a specific muscle.
- (nullable MusclePathData *)musclePathDataForName:(NSString *)name;

/// All muscle names.
@property (nonatomic, readonly) NSArray<NSString *> *muscleNames;

@end

NS_ASSUME_NONNULL_END
