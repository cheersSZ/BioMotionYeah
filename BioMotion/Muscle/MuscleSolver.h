#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

/// Result from muscle static optimization.
@interface MuscleActivationResult : NSObject
@property (nonatomic, readonly) NSArray<NSString *> *muscleNames;
@property (nonatomic, readonly) NSArray<NSNumber *> *activations;  // 0-1 per muscle
@property (nonatomic, readonly) NSArray<NSNumber *> *forces;       // Newtons per muscle
@property (nonatomic, readonly) double solveTimeMs;
@property (nonatomic, readonly) BOOL converged;
@end

/// Solves the muscle static optimization problem:
/// Given joint torques, find muscle activations that minimize sum(a^2)
/// subject to moment arm constraints and activation bounds.
@interface MuscleSolver : NSObject

/// Parse muscle definitions from a .osim model file.
/// @param path Path to the .osim file.
/// @return YES if muscles were parsed successfully.
- (BOOL)loadMusclesFromOsimPath:(NSString *)path;

/// Number of muscles loaded.
@property (nonatomic, readonly) NSInteger numMuscles;

/// Names of all loaded muscles.
@property (nonatomic, readonly) NSArray<NSString *> *muscleNames;

/// Solve static optimization: given joint torques and current skeleton state,
/// find optimal muscle activations.
/// @param jointTorques Array of joint torques (Nm) from inverse dynamics.
/// @param jointAngles Current joint angles (rad) from IK.
/// @param jointVelocities Current joint velocities (rad/s).
/// @param dofNames DOF names corresponding to the torque/angle arrays.
/// @return Muscle activations and forces, or nil on failure.
- (nullable MuscleActivationResult *)solveWithJointTorques:(NSArray<NSNumber *> *)jointTorques
                                               jointAngles:(NSArray<NSNumber *> *)jointAngles
                                           jointVelocities:(NSArray<NSNumber *> *)jointVelocities
                                                  dofNames:(NSArray<NSString *> *)dofNames;

@end

NS_ASSUME_NONNULL_END
