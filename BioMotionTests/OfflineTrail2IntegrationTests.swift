import XCTest
@testable import BioMotion

/// End-to-end validation that the offline pipeline produces biomechanically
/// sensible numbers on the bundled trail2 fixture (LaiUhlrich2022_scaled.osim
/// + run2.mot).
///
/// What this test guards against:
///   - silent breakage of the .mot → DOF mapping in `ImportedMotionPreparer`
///   - regression of the SG filter warm-up math (no captured frames)
///   - blow-ups in inverse dynamics (unbounded torques) caused by bad ddq
///   - the muscle solver failing to converge on a large fraction of frames
///
/// Sub-sampled to keep CI time bounded — see `frameStride`. Full 3823-frame
/// run takes ~30 s on the simulator; 100-frame slice runs in <2 s.
final class OfflineTrail2IntegrationTests: XCTestCase {

    /// Tested every Nth motion frame to keep the test fast while still
    /// exercising the full IK→ID→SO loop end-to-end.
    private let frameStride = 38   // ~100 of 3823 frames

    @MainActor
    func testTrail2BatchAnalysisProducesSaneNumbers() throws {
        let bundle = Bundle(for: type(of: self))
        guard let modelPath = bundle.path(forResource: "LaiUhlrich2022_scaled",
                                          ofType: "osim",
                                          inDirectory: "trail2") else {
            throw XCTSkip("trail2/LaiUhlrich2022_scaled.osim not bundled — skipping")
        }
        guard let motPath = bundle.path(forResource: "run2",
                                        ofType: "mot",
                                        inDirectory: "trail2") else {
            throw XCTSkip("trail2/run2.mot not bundled — skipping")
        }

        // Load model.
        let bridge = NimbleBridge()
        XCTAssertTrue(bridge.loadModel(fromPath: modelPath),
                      "should load LaiUhlrich2022_scaled.osim")
        XCTAssertGreaterThan(bridge.numDOFs, 20)
        XCTAssertGreaterThan(bridge.totalMass, 30, "scaled model mass should be > 30 kg")

        // Parse + prepare motion.
        let motURL = URL(fileURLWithPath: motPath)
        let series = try MOTParser.parse(url: motURL)
        XCTAssertGreaterThan(series.frames.count, 1000)
        let prepared = ImportedMotionPreparer.prepare(
            series: series,
            modelDOFNames: bridge.dofNames as [String]
        )
        XCTAssertEqual(prepared.frames.count, series.frames.count)
        XCTAssertEqual(prepared.unmatchedSourceCoordinates, [],
                       "all .mot coordinates should map into the .osim")

        // Spin up the engine and replay a strided slice of the motion through
        // the same code path the offline session uses (processImportedMotionFrame
        // → SG filter → ID → SO), then capture the result.
        let engine = NimbleEngine()
        let loaded = expectation(description: "engine load model")
        engine.loadModel(fromPath: modelPath) { ok in
            XCTAssertTrue(ok)
            loaded.fulfill()
        }
        wait(for: [loaded], timeout: 30)

        engine.startRecordingResults()

        let strided = stride(from: 0, to: prepared.frames.count, by: frameStride)
            .map { prepared.frames[$0] }
        XCTAssertGreaterThan(strided.count, 30,
                             "need at least ~30 frames after striding to warm up SG filter")

        for frame in strided {
            engine.processImportedMotionFrame(frame)
        }
        engine.waitForPendingSolverWork()
        engine.stopRecordingResults()

        let track = OfflineAnalysisTrack.capture(
            from: engine,
            dofNames: prepared.modelDOFNames,
            modelName: "LaiUhlrich2022_scaled.osim",
            motionName: "run2.mot"
        )

        // --- Captured frames sanity ---
        // SG filter eats the first ~9 frames of warm-up + outputs ~4 frames of
        // lag, so capture is shorter than input by ~8.
        XCTAssertGreaterThan(track.frameCount, strided.count - 12,
                             "captured frame count should match input minus SG warm-up")

        // --- Mass / DOF sanity ---
        XCTAssertGreaterThan(track.totalMassKg, 30)
        XCTAssertEqual(track.dofNames.count, prepared.modelDOFNames.count)

        // --- Joint torques in physiological band ---
        // Running task max joint torque per kg sits around 1–4 Nm/kg for an
        // adult subject. Anything > 10 Nm/kg means GRF wasn't applied or ddq is
        // exploding. Allow a generous upper bound to cover impact peaks.
        let maxTorquePerKg = track.maxTorquePerKg
        XCTAssertGreaterThan(maxTorquePerKg, 0.05, "torques should be non-trivial")
        XCTAssertLessThan(maxTorquePerKg, 30,
                          "max |τ|/kg = \(maxTorquePerKg) Nm/kg is implausible — pipeline is producing garbage")

        // --- Root residual sanity (only meaningful if any frame has a contact) ---
        // < ~0.5 Nm/kg in steady stance is the goal. Allow up to 5 Nm/kg as a
        // crash-not-disaster ceiling for offline mode where GRF heuristics may
        // not perfectly match the trial.
        let anyContact = track.samples.contains { $0.leftFootInContact || $0.rightFootInContact }
        if anyContact {
            XCTAssertLessThan(track.maxRootResidualPerKg, 5.0,
                              "root residual = \(track.maxRootResidualPerKg) Nm/kg suggests GRF inconsistent with kinematics")
        }

        // --- Muscle solver convergence ---
        if !track.muscleNames.isEmpty {
            let convergedFrames = track.samples.filter(\.muscleConverged).count
            let convergedFraction = Double(convergedFrames) / Double(track.frameCount)
            XCTAssertGreaterThan(convergedFraction, 0.5,
                                 "muscle solver converged on \(convergedFrames)/\(track.frameCount) frames — investigation required")

            // Activations are bounded by construction; verify nothing escaped.
            for sample in track.samples {
                for (name, value) in sample.activations {
                    XCTAssert(value.isFinite, "non-finite activation for \(name): \(value)")
                    XCTAssert(value >= -1e-6 && value <= 1.0 + 1e-6,
                              "activation out of [0,1] for \(name): \(value)")
                }
            }
        }
    }
}
