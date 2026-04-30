import Foundation

/// One processed offline frame: smoothed kinematics passed in, plus everything
/// the post-IK pipeline produced for that frame.
///
/// Timestamps are the SG filter's *center* timestamp, so successive frames are
/// monotonically increasing and lag the input by ~4 frames once warmed up.
struct OfflineFrameSample: Codable {
    let timestamp: TimeInterval
    let jointAngles: [String: Double]      // radians
    let jointTorques: [String: Double]     // Nm
    let activations: [String: Double]      // 0...1
    let muscleForces: [String: Double]     // N
    let leftFootForce: SIMD3<Double>       // N (world)
    let rightFootForce: SIMD3<Double>
    let leftFootInContact: Bool
    let rightFootInContact: Bool
    let rootResidualNorm: Double           // Nm
    let maxAbsTorqueNm: Double             // Nm
    let muscleConverged: Bool
    let ikError: Double                    // m (will be 0 in offline mode — we skip IK)
}

/// All-frames trajectory captured by `OfflineSession.processAllFrames()`.
///
/// This is the offline analog of the online `MotionRecorder` but pulls from
/// `NimbleEngine.dynamicsHistory` rather than ARKit, so it carries everything
/// needed for `.mot` / `.sto` export and for plotting.
struct OfflineAnalysisTrack: Codable {
    let modelName: String?
    let motionName: String?
    let totalMassKg: Double
    let dofNames: [String]
    let muscleNames: [String]
    let samples: [OfflineFrameSample]
    /// Inverse-dynamics path that produced this track. `"withGRF"` (default,
    /// production) or `"noGRF"` (diagnostic A/B run). Stored as a string so
    /// the JSON wire format stays human-inspectable; round-trips through
    /// Codable transparently. Older blobs without the key decode to
    /// `"withGRF"` for back-compat.
    let idMode: String

    var frameCount: Int { samples.count }

    var duration: TimeInterval {
        guard let first = samples.first, let last = samples.last else { return 0 }
        return max(0, last.timestamp - first.timestamp)
    }

    var fps: Double {
        guard samples.count >= 2 else { return 0 }
        let dt = samples[1].timestamp - samples[0].timestamp
        guard dt > 0 else { return 0 }
        return 1.0 / dt
    }

    /// Maximum |joint torque| across the whole clip, normalized by body mass.
    /// Physiological reference for walking/running: ~1–3 Nm/kg. Anything > ~10
    /// usually means GRF is missing or kinematics derivatives are corrupted.
    var maxTorquePerKg: Double {
        guard totalMassKg > 0 else { return 0 }
        return (samples.map(\.maxAbsTorqueNm).max() ?? 0) / totalMassKg
    }

    /// Maximum root 6D residual across the clip, normalized by body mass.
    /// < ~0.5 Nm/kg means GRF is consistent with the imported kinematics.
    var maxRootResidualPerKg: Double {
        guard totalMassKg > 0 else { return 0 }
        return (samples.map(\.rootResidualNorm).max() ?? 0) / totalMassKg
    }

    /// Mean activation across the clip per muscle, returned as a sorted list.
    func meanActivationsByMuscle() -> [(name: String, mean: Double)] {
        guard !samples.isEmpty else { return [] }
        var sums: [String: Double] = [:]
        var counts: [String: Int] = [:]
        for sample in samples {
            for (name, value) in sample.activations {
                sums[name, default: 0] += value
                counts[name, default: 0] += 1
            }
        }
        return sums
            .map { (name: $0.key, mean: $0.value / Double(max(counts[$0.key] ?? 1, 1))) }
            .sorted(by: { $0.mean > $1.mean })
    }

    private enum CodingKeys: String, CodingKey {
        case modelName, motionName, totalMassKg, dofNames,
             muscleNames, samples, idMode
    }

    /// Custom decoder so JSON written by older builds (which lacked
    /// `idMode`) still rehydrates cleanly — those tracks were always
    /// `withGRF` at the time, so that's the safe default. All other fields
    /// remain required.
    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        modelName = try c.decodeIfPresent(String.self, forKey: .modelName)
        motionName = try c.decodeIfPresent(String.self, forKey: .motionName)
        totalMassKg = try c.decode(Double.self, forKey: .totalMassKg)
        dofNames = try c.decode([String].self, forKey: .dofNames)
        muscleNames = try c.decode([String].self, forKey: .muscleNames)
        samples = try c.decode([OfflineFrameSample].self, forKey: .samples)
        idMode = try c.decodeIfPresent(String.self, forKey: .idMode) ?? "withGRF"
    }

    /// Memberwise init kept explicit because adding `init(from:)` above
    /// suppresses Swift's synthesized initializer.
    init(
        modelName: String?,
        motionName: String?,
        totalMassKg: Double,
        dofNames: [String],
        muscleNames: [String],
        samples: [OfflineFrameSample],
        idMode: String
    ) {
        self.modelName = modelName
        self.motionName = motionName
        self.totalMassKg = totalMassKg
        self.dofNames = dofNames
        self.muscleNames = muscleNames
        self.samples = samples
        self.idMode = idMode
    }

    /// Convenience builder used by `OfflineSession` after a batch run.
    /// Reads from the engine's recorded histories — see `NimbleEngine.startRecordingResults`.
    @MainActor
    static func capture(
        from engine: NimbleEngine,
        dofNames: [String],
        modelName: String?,
        motionName: String?,
        idMode: NimbleEngine.IDMode
    ) -> OfflineAnalysisTrack {
        // ikHistory & dynamicsHistory are appended in the same pass through the
        // pipeline, so they line up index-for-index. Use the shorter as the
        // safety bound in case one path skipped a frame.
        let ik = engine.ikHistory
        let dyn = engine.dynamicsHistory
        let count = min(ik.count, dyn.count)

        var samples: [OfflineFrameSample] = []
        samples.reserveCapacity(count)

        var muscleNameSet = Set<String>()

        for i in 0..<count {
            let ikEntry = ik[i]
            let dynEntry = dyn[i]
            let activations = dynEntry.muscle?.activations ?? [:]
            let forces = dynEntry.muscle?.forces ?? [:]
            for name in activations.keys { muscleNameSet.insert(name) }

            samples.append(
                OfflineFrameSample(
                    timestamp: dynEntry.timestamp,
                    jointAngles: ikEntry.angles,
                    jointTorques: engine.idHistory[safe: i]?.jointTorques ?? [:],
                    activations: activations,
                    muscleForces: forces,
                    leftFootForce: dynEntry.leftFootForce,
                    rightFootForce: dynEntry.rightFootForce,
                    leftFootInContact: dynEntry.leftFootInContact,
                    rightFootInContact: dynEntry.rightFootInContact,
                    rootResidualNorm: dynEntry.rootResidualNorm,
                    maxAbsTorqueNm: dynEntry.maxAbsTorqueNm,
                    muscleConverged: dynEntry.muscle?.converged ?? false,
                    ikError: ikEntry.error
                )
            )
        }

        return OfflineAnalysisTrack(
            modelName: modelName,
            motionName: motionName,
            totalMassKg: engine.totalMassKg,
            dofNames: dofNames,
            muscleNames: muscleNameSet.sorted(),
            samples: samples,
            idMode: (idMode == .noGRF) ? "noGRF" : "withGRF"
        )
    }
}

private extension Array {
    subscript(safe index: Int) -> Element? {
        guard indices.contains(index) else { return nil }
        return self[index]
    }
}
