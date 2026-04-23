import XCTest
@testable import BioMotion

final class OfflineAnalysisExporterTests: XCTestCase {

    // MARK: - Track-shape sanity

    func testEmptyTrackThrows() {
        let track = makeTrack(samples: [])
        XCTAssertThrowsError(try OfflineAnalysisExporter.export(track: track, stem: "x")) { error in
            guard case OfflineAnalysisExporter.ExportError.noFrames = error else {
                return XCTFail("expected noFrames, got \(error)")
            }
        }
    }

    // MARK: - .mot kinematics formatting

    func testKinematicsMotConvertsRotationsToDegreesButLeavesTranslations() {
        let sample = OfflineFrameSample(
            timestamp: 0.50,
            jointAngles: [
                "pelvis_tilt": .pi,        // 180°
                "pelvis_tx": 1.25,          // meters; must NOT be touched
                "hip_flexion_r": .pi / 2.0  // 90°
            ],
            jointTorques: [:],
            activations: [:],
            muscleForces: [:],
            leftFootForce: .zero,
            rightFootForce: .zero,
            leftFootInContact: false,
            rightFootInContact: false,
            rootResidualNorm: 0,
            maxAbsTorqueNm: 0,
            muscleConverged: false,
            ikError: 0
        )
        let track = makeTrack(
            samples: [sample],
            dofNames: ["pelvis_tilt", "pelvis_tx", "hip_flexion_r"]
        )

        let text = OfflineAnalysisExporter.motTextForKinematics(track: track)
        XCTAssertTrue(text.contains("inDegrees=yes"))
        XCTAssertTrue(text.contains("nRows=1"))

        // Last line is the data row. Time is normalized to start at 0.
        let dataLine = text.split(separator: "\n").last.map(String.init) ?? ""
        let cols = dataLine.split(separator: "\t").map(String.init)
        XCTAssertEqual(cols.count, 4)
        XCTAssertEqual(Double(cols[0])!, 0.0, accuracy: 1e-9)
        XCTAssertEqual(Double(cols[1])!, 180.0, accuracy: 1e-3)   // pelvis_tilt → degrees
        XCTAssertEqual(Double(cols[2])!, 1.25, accuracy: 1e-9)    // pelvis_tx untouched
        XCTAssertEqual(Double(cols[3])!, 90.0, accuracy: 1e-3)    // hip_flexion_r → degrees
    }

    func testInverseDynamicsStoLeavesTorquesInNm() {
        let sample = OfflineFrameSample(
            timestamp: 1.0,
            jointAngles: [:],
            jointTorques: ["hip_flexion_r": 12.5, "knee_angle_r": -7.25],
            activations: [:],
            muscleForces: [:],
            leftFootForce: .zero,
            rightFootForce: .zero,
            leftFootInContact: false,
            rightFootInContact: false,
            rootResidualNorm: 0,
            maxAbsTorqueNm: 12.5,
            muscleConverged: false,
            ikError: 0
        )
        let track = makeTrack(
            samples: [sample],
            dofNames: ["hip_flexion_r", "knee_angle_r"]
        )
        let text = OfflineAnalysisExporter.stoTextForInverseDynamics(track: track)
        XCTAssertTrue(text.contains("inDegrees=no"))
        let dataLine = text.split(separator: "\n").last.map(String.init) ?? ""
        let cols = dataLine.split(separator: "\t").map(String.init)
        XCTAssertEqual(cols.count, 3)
        XCTAssertEqual(Double(cols[1])!, 12.5, accuracy: 1e-6)
        XCTAssertEqual(Double(cols[2])!, -7.25, accuracy: 1e-6)
    }

    // MARK: - File output

    func testExportProducesAllExpectedFilesOnDisk() throws {
        let sample = OfflineFrameSample(
            timestamp: 0,
            jointAngles: ["hip_flexion_r": 0.1],
            jointTorques: ["hip_flexion_r": 5.0],
            activations: ["soleus_r": 0.4],
            muscleForces: ["soleus_r": 250.0],
            leftFootForce: .zero,
            rightFootForce: .zero,
            leftFootInContact: false,
            rightFootInContact: false,
            rootResidualNorm: 0.5,
            maxAbsTorqueNm: 5.0,
            muscleConverged: true,
            ikError: 0
        )
        let track = makeTrack(
            samples: [sample],
            dofNames: ["hip_flexion_r"],
            muscleNames: ["soleus_r"],
            totalMassKg: 70
        )

        let output = try OfflineAnalysisExporter.export(
            track: track,
            stem: "unit-test",
            into: FileManager.default.temporaryDirectory
        )

        XCTAssertTrue(FileManager.default.fileExists(atPath: output.kinematicsMotURL.path))
        XCTAssertTrue(FileManager.default.fileExists(atPath: output.inverseDynamicsSTOURL.path))
        XCTAssertNotNil(output.activationsSTOURL)
        XCTAssertNotNil(output.muscleForcesSTOURL)
        XCTAssertTrue(FileManager.default.fileExists(atPath: output.activationsSTOURL!.path))
        XCTAssertTrue(FileManager.default.fileExists(atPath: output.muscleForcesSTOURL!.path))
        XCTAssertTrue(FileManager.default.fileExists(atPath: output.summaryJSONURL.path))

        // Summary JSON should be parseable and carry the headline fields.
        let data = try Data(contentsOf: output.summaryJSONURL)
        let json = try XCTUnwrap(JSONSerialization.jsonObject(with: data) as? [String: Any])
        XCTAssertEqual(json["frame_count"] as? Int, 1)
        XCTAssertEqual(json["total_mass_kg"] as? Double, 70)
        XCTAssertEqual(json["muscle_count"] as? Int, 1)
        XCTAssertEqual(json["muscle_converged_frames"] as? Int, 1)

        try? FileManager.default.removeItem(at: output.folder)
    }

    // MARK: - Track aggregates

    func testMaxTorquePerKgAndRootResidualPerKg() {
        let s1 = makeSample(timestamp: 0,   maxAbsTorqueNm: 30,  rootResidualNorm: 5)
        let s2 = makeSample(timestamp: 0.1, maxAbsTorqueNm: 70,  rootResidualNorm: 20)
        let track = makeTrack(samples: [s1, s2], totalMassKg: 70)
        XCTAssertEqual(track.maxTorquePerKg, 1.0, accuracy: 1e-9)
        XCTAssertEqual(track.maxRootResidualPerKg, 20.0 / 70.0, accuracy: 1e-9)
    }

    func testMeanActivationsByMuscleSortsHighToLow() {
        let s1 = makeSample(timestamp: 0,   activations: ["a": 0.9, "b": 0.1])
        let s2 = makeSample(timestamp: 0.1, activations: ["a": 0.7, "b": 0.3])
        let track = makeTrack(samples: [s1, s2])
        let ranked = track.meanActivationsByMuscle()
        XCTAssertEqual(ranked.map(\.name), ["a", "b"])
        XCTAssertEqual(ranked[0].mean, 0.8, accuracy: 1e-9)
        XCTAssertEqual(ranked[1].mean, 0.2, accuracy: 1e-9)
    }

    // MARK: - Helpers

    private func makeTrack(
        samples: [OfflineFrameSample],
        dofNames: [String] = [],
        muscleNames: [String] = [],
        totalMassKg: Double = 0
    ) -> OfflineAnalysisTrack {
        OfflineAnalysisTrack(
            modelName: "test.osim",
            motionName: "test.mot",
            totalMassKg: totalMassKg,
            dofNames: dofNames,
            muscleNames: muscleNames,
            samples: samples
        )
    }

    private func makeSample(
        timestamp: TimeInterval,
        activations: [String: Double] = [:],
        maxAbsTorqueNm: Double = 0,
        rootResidualNorm: Double = 0
    ) -> OfflineFrameSample {
        OfflineFrameSample(
            timestamp: timestamp,
            jointAngles: [:],
            jointTorques: [:],
            activations: activations,
            muscleForces: [:],
            leftFootForce: .zero,
            rightFootForce: .zero,
            leftFootInContact: false,
            rightFootInContact: false,
            rootResidualNorm: rootResidualNorm,
            maxAbsTorqueNm: maxAbsTorqueNm,
            muscleConverged: false,
            ikError: 0
        )
    }
}
