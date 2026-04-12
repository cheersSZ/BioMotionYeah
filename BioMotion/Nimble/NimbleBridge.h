#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

/// Result from inverse kinematics solve.
@interface NimbleIKResult : NSObject
@property (nonatomic, readonly) NSArray<NSNumber *> *jointAngles;  // DOF values in radians
@property (nonatomic, readonly) double error;  // RMS marker error in meters
@property (nonatomic, readonly) NSInteger numDOFs;
@property (nonatomic, readonly) NSArray<NSString *> *dofNames;
@end

/// Result from inverse dynamics solve.
@interface NimbleIDResult : NSObject
@property (nonatomic, readonly) NSArray<NSNumber *> *jointTorques;  // Nm per DOF

// Ground reaction force diagnostics (valid only for ID-with-GRF variants).
// World-frame 3-vectors in newtons and meters respectively. Zero vectors when
// the foot was not in contact on this frame.
@property (nonatomic, readonly) NSArray<NSNumber *> *leftFootForce;   // [fx, fy, fz] N
@property (nonatomic, readonly) NSArray<NSNumber *> *rightFootForce;  // [fx, fy, fz] N
@property (nonatomic, readonly) NSArray<NSNumber *> *leftFootCoP;     // [x, y, z] m
@property (nonatomic, readonly) NSArray<NSNumber *> *rightFootCoP;    // [x, y, z] m
@property (nonatomic, readonly) BOOL leftFootInContact;
@property (nonatomic, readonly) BOOL rightFootInContact;
/// Norm of the 6D residual at the free (root) joint — a measure of how well
/// the GRF solution reconciles with the inertial state. Should be small
/// (< ~10 Nm total) when GRF is correctly estimated. Large values mean the
/// contact model is wrong (wrong ground height, missed contact, airborne).
@property (nonatomic, readonly) double rootResidualNorm;
@end

/// C++ bridge to nimblephysics IK and ID solvers.
@interface NimbleBridge : NSObject

/// Load an OpenSim .osim model from a file path.
/// @param path Path to the .osim file.
/// @return YES if the model was loaded successfully.
- (BOOL)loadModelFromPath:(NSString *)path;

/// Get the number of degrees of freedom in the loaded model.
@property (nonatomic, readonly) NSInteger numDOFs;

/// Get the names of all DOFs in the loaded model.
@property (nonatomic, readonly) NSArray<NSString *> *dofNames;

/// Get marker names defined in the loaded model.
@property (nonatomic, readonly) NSArray<NSString *> *markerNames;

/// Scale the model to match a person's body proportions.
/// @param height Body height in meters.
/// @param markerPositions Flat array of marker 3D positions [x0,y0,z0, x1,y1,z1, ...] in meters.
/// @param markerNames Names of the markers corresponding to positions.
- (BOOL)scaleModelWithHeight:(double)height
             markerPositions:(NSArray<NSNumber *> *)markerPositions
                 markerNames:(NSArray<NSString *> *)markerNames;

/// Run inverse kinematics: given 3D marker positions, solve for joint angles.
/// @param markerPositions Flat array of marker 3D positions [x0,y0,z0, x1,y1,z1, ...] in meters.
/// @param markerNames Names of the markers corresponding to positions.
/// @return IK result with joint angles and error, or nil on failure.
- (nullable NimbleIKResult *)solveIKWithMarkerPositions:(NSArray<NSNumber *> *)markerPositions
                                            markerNames:(NSArray<NSString *> *)markerNames;

/// Run inverse dynamics: given joint angles and accelerations, solve for joint torques.
/// @param jointAngles Current joint angles (from IK).
/// @param jointVelocities Current joint velocities (finite difference from IK).
/// @param jointAccelerations Current joint accelerations (finite difference from velocities).
/// @return ID result with joint torques, or nil on failure.
- (nullable NimbleIDResult *)solveIDWithJointAngles:(NSArray<NSNumber *> *)jointAngles
                                   jointVelocities:(NSArray<NSNumber *> *)jointVelocities
                               jointAccelerations:(NSArray<NSNumber *> *)jointAccelerations;

/// Run inverse dynamics with automatic ground reaction force estimation.
///
/// Detects which feet are in contact with the ground (based on `calcn_l` and
/// `calcn_r` body position versus the current ground height) and uses
/// Nimble's multi-contact near-CoP ID solver to decompose the system wrench
/// into per-foot GRFs + joint torques. This is the physically correct way
/// to run ID for any scenario where the subject has ground contact (standing,
/// walking, squatting, sit-to-stand). Use `solveIDWithJointAngles:...` only
/// for pure flight-phase motions.
///
/// @param jointAngles       Smoothed joint angles from IK (q).
/// @param jointVelocities   Smoothed joint velocities (dq), temporally aligned with q.
/// @param jointAccelerations Smoothed joint accelerations (ddq), aligned with q.
/// @return An NimbleIDResult with jointTorques populated plus per-foot
///         force / CoP / contact-state fields. Returns nil on failure.
- (nullable NimbleIDResult *)solveIDGRFWithJointAngles:(NSArray<NSNumber *> *)jointAngles
                                       jointVelocities:(NSArray<NSNumber *> *)jointVelocities
                                    jointAccelerations:(NSArray<NSNumber *> *)jointAccelerations;

/// Sets the ground plane y-coordinate in the ARKit world frame. Call once
/// after calibration (or let it auto-calibrate from the first valid frame).
/// Ground is assumed flat; M4+ may support tilted floors.
- (void)setGroundHeightY:(double)y;

/// Current ground height used for contact detection.
@property (nonatomic, readonly) double groundHeightY;

/// Whether a ground height has been explicitly set or auto-calibrated.
@property (nonatomic, readonly) BOOL groundHeightCalibrated;

/// Whether a model is currently loaded.
@property (nonatomic, readonly) BOOL isModelLoaded;

/// Total mass of the current skeleton (after scaling), in kilograms.
/// Returns 0 if no model is loaded.
@property (nonatomic, readonly) double totalMass;

@end

NS_ASSUME_NONNULL_END
