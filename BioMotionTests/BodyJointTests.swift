import XCTest
@testable import BioMotion

final class BodyJointTests: XCTestCase {

    // MARK: - TrackedJoint

    func testTrackedJointProperties() {
        let joint = TrackedJoint(
            id: "hips_joint",
            name: "Pelvis",
            worldPosition: SIMD3<Float>(1.0, 2.0, 3.0),
            isTracked: true
        )
        XCTAssertEqual(joint.id, "hips_joint")
        XCTAssertEqual(joint.name, "Pelvis")
        XCTAssertEqual(joint.worldPosition.x, 1.0, accuracy: 0.001)
        XCTAssertEqual(joint.worldPosition.y, 2.0, accuracy: 0.001)
        XCTAssertEqual(joint.worldPosition.z, 3.0, accuracy: 0.001)
        XCTAssertTrue(joint.isTracked)
    }

    func testUntrackedJoint() {
        let joint = TrackedJoint(
            id: "left_hand_joint",
            name: "L Wrist",
            worldPosition: .zero,
            isTracked: false
        )
        XCTAssertFalse(joint.isTracked)
        XCTAssertEqual(joint.worldPosition, .zero)
    }

    // MARK: - BodyFrame

    func testBodyFrameRootPosition() {
        let joints = [
            TrackedJoint(id: "hips_joint", name: "Pelvis", worldPosition: SIMD3(0.5, 1.0, 0.2), isTracked: true),
            TrackedJoint(id: "left_upLeg_joint", name: "L Hip", worldPosition: SIMD3(0.4, 0.9, 0.2), isTracked: true),
        ]
        let frame = BodyFrame(timestamp: 0.0, frameNumber: 1, joints: joints)

        XCTAssertNotNil(frame.rootPosition)
        XCTAssertEqual(Double(frame.rootPosition!.x), 0.5, accuracy: 0.001)
        XCTAssertEqual(Double(frame.rootPosition!.y), 1.0, accuracy: 0.001)
    }

    func testBodyFrameRootPositionMissing() {
        let joints = [
            TrackedJoint(id: "left_upLeg_joint", name: "L Hip", worldPosition: SIMD3(0.4, 0.9, 0.2), isTracked: true),
        ]
        let frame = BodyFrame(timestamp: 0.0, frameNumber: 1, joints: joints)
        XCTAssertNil(frame.rootPosition)
    }

    // MARK: - JointMapping

    func testPrimaryMappingCount() {
        XCTAssertEqual(JointMapping.primary.count, 20, "Should have 20 primary joint mappings")
    }

    func testPrimaryMappingHasUniqueARKitNames() {
        let arkitNames = JointMapping.primary.map(\.arkitName)
        let uniqueNames = Set(arkitNames)
        XCTAssertEqual(arkitNames.count, uniqueNames.count, "ARKit joint names must be unique")
    }

    func testPrimaryMappingHasUniqueOpenSimNames() {
        let opensimNames = JointMapping.primary.map(\.opensimName)
        let uniqueNames = Set(opensimNames)
        XCTAssertEqual(opensimNames.count, uniqueNames.count, "OpenSim marker names must be unique")
    }

    func testBonesReferenceValidIndices() {
        let maxIndex = JointMapping.primary.count - 1
        for (i, bone) in JointMapping.bones.enumerated() {
            XCTAssertGreaterThanOrEqual(bone.0, 0, "Bone \(i) start index out of range")
            XCTAssertLessThanOrEqual(bone.0, maxIndex, "Bone \(i) start index out of range")
            XCTAssertGreaterThanOrEqual(bone.1, 0, "Bone \(i) end index out of range")
            XCTAssertLessThanOrEqual(bone.1, maxIndex, "Bone \(i) end index out of range")
        }
    }

    func testKnownMappings() {
        let pelvis = JointMapping.primary.first(where: { $0.arkitName == "hips_joint" })
        XCTAssertNotNil(pelvis)
        XCTAssertEqual(pelvis?.opensimName, "PELVIS")

        let rightKnee = JointMapping.primary.first(where: { $0.arkitName == "right_leg_joint" })
        XCTAssertNotNil(rightKnee)
        XCTAssertEqual(rightKnee?.opensimName, "RKJC")
    }
}
