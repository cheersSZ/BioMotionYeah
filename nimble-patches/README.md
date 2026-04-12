# Nimble patches

This folder holds local patches to `nimblephysics/` that are required for
BioMotion to build and run correctly but haven't been upstreamed.
`nimblephysics/` itself is gitignored (it's a vendored clone), so these
patches serve as the authoritative record of what we changed.

## `opensimparser-null-joint-fallback.patch`

**Applies to**: `nimblephysics/dart/biomechanics/OpenSimParser.cpp`
**Introduced in**: build 13 (BioMotion accuracy-overhaul branch)

### Problem

`dart::biomechanics::createJoint()` has several paths where it uses
`assert(false && "...")` to signal that it couldn't construct a joint —
for example when a `CustomJoint`'s `dofNames.size()` doesn't match any
of the 1-6 `createCustomJoint<N>` specializations, or when a
`TransformAxis` uses a function type the parser doesn't recognize.

In **release** builds `assert()` is a no-op, so those paths fall
through with the local `joint` variable left as `nullptr`. Later, at
line 5779, `joint->setName(jointName)` dereferences a null `this` and
the app segfaults at `this + 0x3f` inside `Joint::setName`.

This was discovered when swapping the bundled osim to
`cyclistFullBodyMuscle.osim`, which has 53 CustomJoints — at least one
triggers the uncovered branch and takes the whole parse down.

### Fix

Before `joint->setName(jointName)`, check for null and substitute a
`WeldJoint` in place. Log the failing joint name + type to stderr so
we can later decide whether to (a) extend nimble's parser to handle
that joint type or (b) pre-process the XML.

```cpp
if (joint == nullptr) {
    std::cerr << "OpenSimParser: failed to construct joint \""
              << jointName << "\" of type \"" << jointType << "\" "
              << "— substituting a WeldJoint so parse can continue."
              << std::endl;
    dynamics::WeldJoint::Properties props;
    props.mName = jointName;
    // create through skel/parentBody as usual...
}
```

The affected joint becomes immobile in the kinematic chain, but the
skeleton structure stays valid and the rest of the model keeps working.

### How to apply after a fresh `nimblephysics/` clone

```sh
cd labs/BioMotion/nimblephysics
git apply ../nimble-patches/opensimparser-null-joint-fallback.patch
cd build_ios && cmake --build .   # rebuild static archive for device
cd ../build_sim && cmake --build .   # rebuild for simulator
```

After that the BioMotion Xcode build will link against the patched
`libnimble_ios.a` automatically because the project.yml points at
`nimblephysics/build_ios/libnimble_ios.a`.
