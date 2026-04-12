import XCTest
@testable import BioMotion

final class NimbleBridgeTests: XCTestCase {

    private var bridge: NimbleBridge!

    override func setUp() {
        super.setUp()
        bridge = NimbleBridge()
    }

    // MARK: - Model Loading

    func testLoadRajagopalModel() {
        let path = Bundle(for: type(of: self)).path(forResource: "Rajagopal2016", ofType: "osim")
            ?? Bundle.main.path(forResource: "Rajagopal2016", ofType: "osim")
        XCTAssertNotNil(path, "Rajagopal2016.osim must be in the test bundle or app bundle")

        guard let path else { return }
        let success = bridge.loadModel(fromPath: path)
        XCTAssertTrue(success, "Model should load successfully")
        XCTAssertTrue(bridge.isModelLoaded)
    }

    func testModelDOFCount() {
        loadModel()
        // Rajagopal2016 has 39 coordinates (some locked)
        XCTAssertGreaterThan(bridge.numDOFs, 20, "Should have >20 DOFs")
        XCTAssertLessThanOrEqual(bridge.numDOFs, 39, "Should have <=39 DOFs")
    }

    func testModelDOFNames() {
        loadModel()
        let names = bridge.dofNames
        XCTAssertFalse(names.isEmpty)

        // Check for known DOF names
        XCTAssertTrue(names.contains(where: { $0.contains("hip_flexion") }),
                      "Should contain hip_flexion DOF")
        XCTAssertTrue(names.contains(where: { $0.contains("knee_angle") }),
                      "Should contain knee_angle DOF")
    }

    func testModelMarkerNames() {
        loadModel()
        let markers = bridge.markerNames
        XCTAssertFalse(markers.isEmpty, "Should have markers")
        // Rajagopal2016 has 66 markers
        XCTAssertGreaterThan(markers.count, 30, "Should have >30 markers")
    }

    // MARK: - Inverse Kinematics

    func testIKWithSyntheticData() {
        loadModel()

        // Create synthetic marker positions (standing pose)
        // Use a subset of markers that exist in the model
        let modelMarkers = bridge.markerNames
        guard modelMarkers.count >= 3 else {
            XCTFail("Need at least 3 markers")
            return
        }

        // Use first 5 markers with plausible 3D positions
        let numMarkers = min(5, modelMarkers.count)
        var positions: [NSNumber] = []
        var names: [String] = []

        for i in 0..<numMarkers {
            names.append(modelMarkers[i])
            // Generic standing position (spread around origin at ~1m height)
            positions.append(NSNumber(value: Double(i) * 0.1 - 0.2))  // x
            positions.append(NSNumber(value: 1.0))                      // y (up)
            positions.append(NSNumber(value: 0.0))                      // z
        }

        let result = bridge.solveIK(withMarkerPositions: positions, markerNames: names)
        XCTAssertNotNil(result, "IK should return a result")

        if let result {
            XCTAssertEqual(result.jointAngles.count, Int(bridge.numDOFs),
                           "Should return angles for all DOFs")
            XCTAssertGreaterThanOrEqual(result.error, 0, "Error should be non-negative")
            XCTAssertEqual(result.dofNames.count, Int(bridge.numDOFs))
        }
    }

    func testIKResultHasReasonableAngles() {
        loadModel()
        let result = runIKWithStandingPose()
        guard let result else { return }

        // Joint angles should be within a reasonable range (< 180 degrees = pi radians)
        for angle in result.jointAngles {
            let value = angle.doubleValue
            XCTAssertLessThan(abs(value), .pi * 2,
                              "Joint angle should be within ±2π radians")
        }
    }

    // MARK: - Inverse Dynamics

    func testIDWithSyntheticData() {
        loadModel()
        let numDOFs = Int(bridge.numDOFs)
        guard numDOFs > 0 else { return }

        // Set up synthetic joint state (small angles, zero velocities/accelerations)
        let angles = Array(repeating: NSNumber(value: 0.0), count: numDOFs)
        let velocities = Array(repeating: NSNumber(value: 0.0), count: numDOFs)
        let accelerations = Array(repeating: NSNumber(value: 0.0), count: numDOFs)

        let result = bridge.solveID(withJointAngles: angles,
                                    jointVelocities: velocities,
                                    jointAccelerations: accelerations)
        XCTAssertNotNil(result, "ID should return a result")

        if let result {
            XCTAssertEqual(result.jointTorques.count, numDOFs,
                           "Should return torques for all DOFs")
            // At zero position/velocity/acceleration, torques should reflect gravity compensation
            let hasNonZeroTorque = result.jointTorques.contains(where: { $0.doubleValue != 0 })
            XCTAssertTrue(hasNonZeroTorque, "Gravity should produce non-zero torques")
        }
    }

    // MARK: - Edge Cases

    func testIKWithEmptyMarkers() {
        loadModel()
        let result = bridge.solveIK(withMarkerPositions: [], markerNames: [])
        XCTAssertNil(result, "Empty markers should return nil")
    }

    func testIKWithMismatchedArrays() {
        loadModel()
        // 2 names but only 3 positions (should be 6 = 2*3)
        let result = bridge.solveIK(
            withMarkerPositions: [1, 2, 3].map { NSNumber(value: $0) },
            markerNames: ["A", "B"]
        )
        XCTAssertNil(result, "Mismatched arrays should return nil")
    }

    func testIDWithWrongDOFCount() {
        loadModel()
        // Pass wrong number of angles
        let result = bridge.solveID(withJointAngles: [NSNumber(value: 0)],
                                    jointVelocities: [NSNumber(value: 0)],
                                    jointAccelerations: [NSNumber(value: 0)])
        XCTAssertNil(result, "Wrong DOF count should return nil")
    }

    func testLoadModelFromInvalidPath() {
        let success = bridge.loadModel(fromPath: "/nonexistent/model.osim")
        XCTAssertFalse(success)
        XCTAssertFalse(bridge.isModelLoaded)
    }

    // MARK: - Helpers

    private func loadModel() {
        let path = Bundle(for: type(of: self)).path(forResource: "Rajagopal2016", ofType: "osim")
            ?? Bundle.main.path(forResource: "Rajagopal2016", ofType: "osim")
        guard let path else {
            XCTFail("Cannot find Rajagopal2016.osim")
            return
        }
        let success = bridge.loadModel(fromPath: path)
        XCTAssertTrue(success)
    }

    private func runIKWithStandingPose() -> NimbleIKResult? {
        let modelMarkers = bridge.markerNames
        let numMarkers = min(5, modelMarkers.count)
        var positions: [NSNumber] = []
        var names: [String] = []

        for i in 0..<numMarkers {
            names.append(modelMarkers[i])
            positions.append(NSNumber(value: Double(i) * 0.1 - 0.2))
            positions.append(NSNumber(value: 1.0))
            positions.append(NSNumber(value: 0.0))
        }

        return bridge.solveIK(withMarkerPositions: positions, markerNames: names)
    }
}
