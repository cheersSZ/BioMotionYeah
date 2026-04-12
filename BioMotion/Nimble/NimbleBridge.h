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

/// Whether a model is currently loaded.
@property (nonatomic, readonly) BOOL isModelLoaded;

@end

NS_ASSUME_NONNULL_END
