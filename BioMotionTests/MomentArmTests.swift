import XCTest
@testable import BioMotion

final class MomentArmTests: XCTestCase {

    private var computer: MomentArmComputer!
    private var bridge: NimbleBridge!

    override func setUp() {
        super.setUp()
        computer = MomentArmComputer()
        bridge = NimbleBridge()
    }

    func testParseMusclePaths() {
        loadModel()
        // Should parse all 80 muscles
        XCTAssertEqual(computer.numMuscles, 80, "Should parse 80 muscle paths")
    }

    func testMusclePathData() {
        loadModel()
        let soleus = computer.musclePathData(forName: "soleus_r")
        XCTAssertNotNil(soleus)
        if let s = soleus {
            XCTAssertEqual(s.name, "soleus_r")
            XCTAssertGreaterThan(s.pathPoints.count, 1, "Soleus should have ≥2 path points")
            XCTAssertGreaterThan(s.maxIsometricForce, 1000, "Soleus F0 should be > 1000N")
            XCTAssertGreaterThan(s.optimalFiberLength, 0.01)
        }
    }

    func testComputeMomentArms() {
        loadModel()
        let dofNames = ["hip_flexion_r", "knee_angle_r", "ankle_angle_r"]
        let angles = dofNames.map { _ in NSNumber(value: 0.0) }

        let result = computer.computeMomentArms(withJointAngles: angles, dofNames: dofNames)
        XCTAssertNotNil(result)

        if let result {
            XCTAssertEqual(result.count, 80 * 3, "Should be nMuscles × nDOFs")

            // Check that SOME moment arms are non-zero
            let nonZero = result.filter { abs($0.doubleValue) > 1e-6 }
            XCTAssertGreaterThan(nonZero.count, 0, "Some moment arms should be non-zero")
        }
    }

    func testSoleusAnkleMomentArm() {
        loadModel()
        let dofNames = ["ankle_angle_r"]
        let angles = [NSNumber(value: 0.0)]

        guard let result = computer.computeMomentArms(withJointAngles: angles, dofNames: dofNames) else {
            XCTFail("Should compute moment arms")
            return
        }

        // Find soleus_r index
        let names = computer.muscleNames
        guard let soleusIdx = (names as [String]).firstIndex(of: "soleus_r") else {
            XCTFail("soleus_r should exist")
            return
        }

        let soleusAnkleMomentArm = result[soleusIdx].doubleValue
        // Soleus should have a substantial moment arm at the ankle (~0.04-0.06m)
        // The sign depends on convention (plantarflexor = negative in OpenSim)
        XCTAssertGreaterThan(abs(soleusAnkleMomentArm), 0.01,
                             "Soleus should have a significant ankle moment arm, got \(soleusAnkleMomentArm)")
    }

    func testMomentArmPerformance() {
        loadModel()
        let dofNames = ["hip_flexion_r", "hip_flexion_l",
                         "knee_angle_r", "knee_angle_l",
                         "ankle_angle_r", "ankle_angle_l"]
        let angles = dofNames.map { _ in NSNumber(value: 0.0) }

        measure {
            _ = computer.computeMomentArms(withJointAngles: angles, dofNames: dofNames)
        }
    }

    func testOneEuroFilter() {
        let filter = OneEuroFilter(minCutoff: 1.0, beta: 0.01)

        // Feed a noisy signal (constant value + noise)
        var outputs: [Double] = []
        for i in 0..<100 {
            let t = Double(i) / 30.0  // 30 fps
            let noise = Double.random(in: -0.01...0.01)
            let value = 1.0 + noise
            outputs.append(filter.filter(value, timestamp: t))
        }

        // After warmup, filtered values should be closer to 1.0 than raw
        let lastOutputs = Array(outputs.suffix(20))
        let avgDeviation = lastOutputs.map { abs($0 - 1.0) }.reduce(0, +) / Double(lastOutputs.count)
        XCTAssertLessThan(avgDeviation, 0.005, "Filter should reduce noise significantly")
    }

    func testOneEuroFilter3D() {
        let filter = OneEuroFilter3D(minCutoff: 1.0, beta: 0.01)

        let smoothed = filter.filter(SIMD3<Float>(1.0, 2.0, 3.0), timestamp: 0.0)
        XCTAssertEqual(smoothed.x, 1.0, accuracy: 0.001)

        // Second sample with noise
        let smoothed2 = filter.filter(SIMD3<Float>(1.05, 2.05, 3.05), timestamp: 0.033)
        // Should be somewhere between the two values (partially filtered)
        XCTAssertGreaterThan(smoothed2.x, 0.99)
        XCTAssertLessThan(smoothed2.x, 1.06)
    }

    // MARK: - Helpers

    private func loadModel() {
        let path = Bundle(for: type(of: self)).path(forResource: "Rajagopal2016", ofType: "osim")
            ?? Bundle.main.path(forResource: "Rajagopal2016", ofType: "osim")
        guard let path else {
            XCTFail("Cannot find Rajagopal2016.osim")
            return
        }
        XCTAssertTrue(bridge.loadModel(fromPath: path))
        let success = computer.parseMusclePaths(fromOsimPath: path, from: bridge)
        XCTAssertTrue(success)
    }
}
