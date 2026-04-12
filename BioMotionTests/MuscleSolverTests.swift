import XCTest
@testable import BioMotion

final class MuscleSolverTests: XCTestCase {

    private var solver: MuscleSolver!

    override func setUp() {
        super.setUp()
        solver = MuscleSolver()
    }

    // MARK: - Model Loading

    func testLoadMusclesFromOsim() {
        loadMuscles()
        XCTAssertGreaterThan(solver.numMuscles, 0, "Should load muscles")
        // Rajagopal2016 has 80 muscles
        XCTAssertEqual(solver.numMuscles, 80, "Rajagopal2016 should have 80 muscles")
    }

    func testMuscleNames() {
        loadMuscles()
        let names = solver.muscleNames
        XCTAssertEqual(names.count, 80)

        // Check for known muscles
        XCTAssertTrue(names.contains("soleus_r"), "Should contain soleus_r")
        XCTAssertTrue(names.contains("soleus_l"), "Should contain soleus_l")
        XCTAssertTrue(names.contains("recfem_r"), "Should contain recfem_r")
        XCTAssertTrue(names.contains("tibant_r"), "Should contain tibant_r")
        XCTAssertTrue(names.contains("gasmed_r"), "Should contain gasmed_r")
    }

    func testBilateralSymmetry() {
        loadMuscles()
        let names = solver.muscleNames

        // Every _r muscle should have a _l counterpart
        let rightMuscles = names.filter { ($0 as String).hasSuffix("_r") }
        for rMuscle in rightMuscles {
            let lMuscle = (rMuscle as String).replacingOccurrences(of: "_r", with: "_l")
            XCTAssertTrue(names.contains(where: { ($0 as String) == lMuscle }),
                          "Missing left counterpart for \(rMuscle)")
        }
    }

    // MARK: - Static Optimization

    func testSolveWithSyntheticTorques() {
        loadMuscles()

        // Simulate a simple hip flexion torque
        let dofNames: [String] = ["hip_flexion_r", "knee_angle_r", "ankle_angle_r"]
        let torques: [NSNumber] = [
            NSNumber(value: 20.0),   // 20 Nm hip flexion
            NSNumber(value: -5.0),   // -5 Nm knee (slight flexion)
            NSNumber(value: -10.0),  // -10 Nm ankle (plantarflexion)
        ]
        let angles = Array(repeating: NSNumber(value: 0.0), count: dofNames.count)
        let velocities = Array(repeating: NSNumber(value: 0.0), count: dofNames.count)

        let result = solver.solve(
            withJointTorques: torques,
            jointAngles: angles,
            jointVelocities: velocities,
            dofNames: dofNames
        )

        XCTAssertNotNil(result, "Muscle solver should return a result")

        if let result {
            XCTAssertEqual(result.muscleNames.count, 80)
            XCTAssertEqual(result.activations.count, 80)
            XCTAssertEqual(result.forces.count, 80)
            XCTAssertGreaterThan(result.solveTimeMs, 0, "Should report solve time")
        }
    }

    func testActivationsInRange() {
        loadMuscles()

        let dofNames: [String] = ["hip_flexion_r", "knee_angle_r"]
        let torques: [NSNumber] = [NSNumber(value: 15.0), NSNumber(value: -10.0)]
        let angles = Array(repeating: NSNumber(value: 0.0), count: 2)
        let velocities = Array(repeating: NSNumber(value: 0.0), count: 2)

        let result = solver.solve(
            withJointTorques: torques,
            jointAngles: angles,
            jointVelocities: velocities,
            dofNames: dofNames
        )

        guard let result else {
            XCTFail("Should produce result")
            return
        }

        for activation in result.activations {
            let a = activation.doubleValue
            XCTAssertGreaterThanOrEqual(a, 0.0, "Activation should be >= 0")
            XCTAssertLessThanOrEqual(a, 1.0, "Activation should be <= 1")
        }
    }

    func testForcesNonNegative() {
        loadMuscles()

        let dofNames: [String] = ["ankle_angle_r"]
        let torques: [NSNumber] = [NSNumber(value: -30.0)]  // Plantarflexion
        let angles = [NSNumber(value: 0.0)]
        let velocities = [NSNumber(value: 0.0)]

        let result = solver.solve(
            withJointTorques: torques,
            jointAngles: angles,
            jointVelocities: velocities,
            dofNames: dofNames
        )

        guard let result else { return }

        for force in result.forces {
            XCTAssertGreaterThanOrEqual(force.doubleValue, 0.0, "Muscle force should be non-negative")
        }
    }

    func testSolvePerformance() {
        loadMuscles()

        let dofNames: [String] = ["hip_flexion_r", "hip_flexion_l",
                                   "knee_angle_r", "knee_angle_l",
                                   "ankle_angle_r", "ankle_angle_l"]
        let torques = dofNames.map { _ in NSNumber(value: Double.random(in: -30...30)) }
        let angles = Array(repeating: NSNumber(value: 0.0), count: dofNames.count)
        let velocities = Array(repeating: NSNumber(value: 0.0), count: dofNames.count)

        measure {
            for _ in 0..<10 {
                _ = solver.solve(
                    withJointTorques: torques,
                    jointAngles: angles,
                    jointVelocities: velocities,
                    dofNames: dofNames
                )
            }
        }
    }

    // MARK: - Edge Cases

    func testSolveWithNoDOFs() {
        loadMuscles()
        let result = solver.solve(withJointTorques: [], jointAngles: [],
                                  jointVelocities: [], dofNames: [])
        // Should handle gracefully (nil or empty result)
    }

    func testLoadFromInvalidPath() {
        let success = solver.loadMuscles(fromOsimPath: "/nonexistent.osim")
        XCTAssertFalse(success)
        XCTAssertEqual(solver.numMuscles, 0)
    }

    // MARK: - Helpers

    private func loadMuscles() {
        let path = Bundle(for: type(of: self)).path(forResource: "Rajagopal2016", ofType: "osim")
            ?? Bundle.main.path(forResource: "Rajagopal2016", ofType: "osim")
        guard let path else {
            XCTFail("Cannot find Rajagopal2016.osim")
            return
        }
        let success = solver.loadMuscles(fromOsimPath: path)
        XCTAssertTrue(success)
    }
}
