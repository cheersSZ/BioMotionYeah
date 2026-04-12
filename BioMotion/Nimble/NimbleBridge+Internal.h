// Internal C++ bridge for sharing the raw Nimble skeleton between ObjC++
// classes inside this target. Do NOT import this header from Swift or from
// non-ObjC++ source files — it leaks the `dart::dynamics::Skeleton` C++ type
// through the ObjC interface and will not compile in pure-ObjC TUs.
//
// The public `NimbleBridge.h` remains pure ObjC and Swift-friendly; this
// category exists so `MomentArmComputer.mm` can reuse the same scaled
// skeleton that NimbleBridge.loadModel / scaleModelWithHeight mutated,
// instead of each ObjC++ class calling `parseOsim` independently and ending
// up with an unscaled duplicate.

#import "NimbleBridge.h"

#include <memory>
#include "dart/dynamics/Skeleton.hpp"

@interface NimbleBridge (Internal)

/// The live skeleton shared_ptr. Returns nullptr if no model is loaded.
/// Any mutation to the returned skeleton (setPositions, setBodyScales, etc.)
/// is visible to every holder of this shared_ptr — which is the point.
- (std::shared_ptr<dart::dynamics::Skeleton>)sharedSkeleton;

@end
