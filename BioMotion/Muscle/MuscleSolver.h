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
///
/// NOTE: this legacy path uses hardcoded moment arms and an idealized
/// Hill model (fiber length = optimal, velocity = 0). Prefer
/// solveWithRealKinematics:... for production use.
- (nullable MuscleActivationResult *)solveWithJointTorques:(NSArray<NSNumber *> *)jointTorques
                                               jointAngles:(NSArray<NSNumber *> *)jointAngles
                                           jointVelocities:(NSArray<NSNumber *> *)jointVelocities
                                                  dofNames:(NSArray<NSString *> *)dofNames;

/// Production muscle static optimization.
///
/// Solves the soft-equality QP:
///   min  ½ aᵀPa + ½ λ ‖R·diag(F_max·f_AL·f_FV·cos(α))·a − τ‖²
///   s.t. a_min ≤ a ≤ 1
///
/// with per-muscle F_max and Hill-model force-length / force-velocity
/// multipliers computed from the REAL current musculotendon length and
/// its first time derivative (both supplied by the caller from a
/// FK-driven moment arm computer). Unlike the legacy solver this one:
///   1. never uses hardcoded moment arms — R comes from outside;
///   2. never goes infeasible — the torque constraint is a soft
///      quadratic penalty rather than an equality;
///   3. computes real normalized fiber length / velocity per frame.
///
/// @param jointTorques    Joint torques from ID (Nm), indexed by dofNames.
/// @param momentArms      Flat row-major [nMuscles × nDOFs] matrix, metres.
/// @param muscleNames     Names of muscles, length nMuscles.
/// @param muscleLengths   Current L_MT(q) for each muscle (m), length nMuscles.
/// @param maxForces       F_max for each muscle (N), length nMuscles.
/// @param optimalFiberLengths l_opt for each muscle (m), length nMuscles.
/// @param tendonSlackLengths  l_Ts for each muscle (m), length nMuscles.
/// @param pennationAngles α₀ for each muscle (rad), length nMuscles.
/// @param jointVelocities dq for each DOF (rad/s), length dofNames.count.
/// @param dofNames        DOF names, ordering matches jointTorques columns.
/// @param dt              Time since last solve (s) — for finite-differencing
///                        L_MT to get d L_MT / dt. Use the SG window dt.
/// @param softPenalty     Soft-equality weight λ. Larger → forces tau match
///                        at the expense of activations; smaller → looser.
- (nullable MuscleActivationResult *)
  solveRealWithJointTorques:(NSArray<NSNumber *> *)jointTorques
                 momentArms:(NSArray<NSNumber *> *)momentArms
                muscleNames:(NSArray<NSString *> *)muscleNames
              muscleLengths:(NSArray<NSNumber *> *)muscleLengths
                  maxForces:(NSArray<NSNumber *> *)maxForces
        optimalFiberLengths:(NSArray<NSNumber *> *)optimalFiberLengths
         tendonSlackLengths:(NSArray<NSNumber *> *)tendonSlackLengths
            pennationAngles:(NSArray<NSNumber *> *)pennationAngles
            jointVelocities:(NSArray<NSNumber *> *)jointVelocities
                   dofNames:(NSArray<NSString *> *)dofNames
                         dt:(double)dt
                softPenalty:(double)softPenalty;

@end

NS_ASSUME_NONNULL_END
