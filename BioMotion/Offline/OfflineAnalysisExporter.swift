import Foundation

/// Serializes an `OfflineAnalysisTrack` to OpenSim-compatible files plus a
/// JSON summary, matching the format other tools in the OpenCap ecosystem
/// expect (degrees for `.mot`, radians/Nm for `.sto`).
enum OfflineAnalysisExporter {

    enum ExportError: Error, LocalizedError {
        case noFrames
        case writeFailed(URL, underlying: Error)

        var errorDescription: String? {
            switch self {
            case .noFrames:
                return "No processed frames to export. Run \"Process All\" first."
            case .writeFailed(let url, let error):
                return "Failed to write \(url.lastPathComponent): \(error.localizedDescription)"
            }
        }
    }

    enum LoadError: Error, LocalizedError {
        case readFailed(URL, underlying: Error)
        case decodeFailed(URL, underlying: Error)
        case unsupportedFormat(actual: Int, supported: Int)

        var errorDescription: String? {
            switch self {
            case .readFailed(let url, let error):
                return "Failed to read \(url.lastPathComponent): \(error.localizedDescription)"
            case .decodeFailed(let url, let error):
                return "Failed to decode \(url.lastPathComponent): \(error.localizedDescription)"
            case .unsupportedFormat(let actual, let supported):
                return "Unsupported processed-analysis file (format v\(actual); this build expects v\(supported))."
            }
        }
    }

    /// Bundle of files produced by a single export. The caller can hand any of
    /// them to `ShareLink` / `UIActivityViewController`.
    struct Output {
        let folder: URL
        let kinematicsMotURL: URL
        let inverseDynamicsSTOURL: URL
        let activationsSTOURL: URL?
        let muscleForcesSTOURL: URL?
        let summaryJSONURL: URL
        /// Lossless JSON of the entire `OfflineAnalysisTrack`. Reload via
        /// `OfflineAnalysisExporter.loadProcessed(from:)` to skip the
        /// IK/ID/SO pipeline on subsequent app launches/installs.
        let processedJSONURL: URL
    }

    /// Bumped whenever `OfflineAnalysisTrack` / `OfflineFrameSample` change
    /// shape in a way that breaks decoding of older blobs.
    static let processedFormatVersion = 1

    /// Top-level wrapper written to `<stem>_processed.json` so the file is
    /// self-describing (carries a format version) instead of a bare track.
    struct ProcessedFile: Codable {
        let formatVersion: Int
        let appVersion: String?
        let exportedAt: Date
        let stem: String
        let track: OfflineAnalysisTrack
    }

    /// Write the four (or six) files into a fresh subfolder of the temp dir.
    /// `stem` is used as the base filename — typically the imported `.mot` name
    /// stripped of its extension.
    @discardableResult
    static func export(
        track: OfflineAnalysisTrack,
        stem: String,
        into directory: URL? = nil
    ) throws -> Output {
        guard !track.samples.isEmpty else { throw ExportError.noFrames }

        let folder = try makeOutputFolder(stem: stem, parent: directory)

        let kinematicsURL = folder.appendingPathComponent("\(stem)_kinematics.mot")
        try write(
            data: motTextForKinematics(track: track),
            to: kinematicsURL
        )

        let idURL = folder.appendingPathComponent("\(stem)_inverse_dynamics.sto")
        try write(
            data: stoTextForInverseDynamics(track: track),
            to: idURL
        )

        var activationsURL: URL?
        var forcesURL: URL?
        if !track.muscleNames.isEmpty {
            let aURL = folder.appendingPathComponent("\(stem)_activations.sto")
            try write(data: stoTextForActivations(track: track), to: aURL)
            activationsURL = aURL

            let fURL = folder.appendingPathComponent("\(stem)_muscle_forces.sto")
            try write(data: stoTextForMuscleForces(track: track), to: fURL)
            forcesURL = fURL
        }

        let summaryURL = folder.appendingPathComponent("\(stem)_summary.json")
        try writeJSON(summary(for: track, stem: stem), to: summaryURL)

        // Lossless round-trip blob. Written alongside the OpenSim files so
        // every export folder is also a "processed cache" that can be loaded
        // back in via `OfflineSession.loadProcessedAnalysis(from:)`.
        let processedURL = folder.appendingPathComponent("\(stem)_processed.json")
        try writeProcessed(track: track, stem: stem, to: processedURL)

        return Output(
            folder: folder,
            kinematicsMotURL: kinematicsURL,
            inverseDynamicsSTOURL: idURL,
            activationsSTOURL: activationsURL,
            muscleForcesSTOURL: forcesURL,
            summaryJSONURL: summaryURL,
            processedJSONURL: processedURL
        )
    }

    // MARK: - Processed-analysis round-trip
    //
    // These two helpers exist so that a one-time `processAllFrames()` run can
    // be replayed across app reinstalls without re-running the pipeline. The
    // file format intentionally mirrors `OfflineAnalysisTrack` 1:1 (via
    // `Codable`) so adding a new per-frame field automatically lights up
    // here — bump `processedFormatVersion` if the change is non-additive.

    /// Encode the track to a lossless JSON blob suitable for re-loading.
    static func encodeProcessed(track: OfflineAnalysisTrack, stem: String) throws -> Data {
        let payload = ProcessedFile(
            formatVersion: processedFormatVersion,
            appVersion: Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String,
            exportedAt: Date(),
            stem: stem,
            track: track
        )
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        encoder.dateEncodingStrategy = .iso8601
        return try encoder.encode(payload)
    }

    /// Decode a previously-exported `<stem>_processed.json` into an
    /// `OfflineAnalysisTrack` ready to assign to `OfflineSession.analysisTrack`.
    static func loadProcessed(from url: URL) throws -> OfflineAnalysisTrack {
        let data: Data
        do {
            data = try Data(contentsOf: url)
        } catch {
            throw LoadError.readFailed(url, underlying: error)
        }
        let decoder = JSONDecoder()
        decoder.dateDecodingStrategy = .iso8601
        let file: ProcessedFile
        do {
            file = try decoder.decode(ProcessedFile.self, from: data)
        } catch {
            throw LoadError.decodeFailed(url, underlying: error)
        }
        guard file.formatVersion == processedFormatVersion else {
            throw LoadError.unsupportedFormat(
                actual: file.formatVersion,
                supported: processedFormatVersion
            )
        }
        return file.track
    }

    private static func writeProcessed(
        track: OfflineAnalysisTrack,
        stem: String,
        to url: URL
    ) throws {
        do {
            let data = try encodeProcessed(track: track, stem: stem)
            try data.write(to: url, options: .atomic)
        } catch let error as ExportError {
            throw error
        } catch {
            throw ExportError.writeFailed(url, underlying: error)
        }
    }

    // MARK: - Mot / Sto writers

    static func motTextForKinematics(track: OfflineAnalysisTrack) -> String {
        // Joint angles → degrees so the file is interchangeable with the
        // OpenCap `.mot` we imported in the first place. Translation DOFs are
        // already in meters and pass through unchanged.
        let dofs = track.dofNames
        let radians = false  // header `inDegrees=yes` will tell consumers what we wrote
        return motOrStoText(
            stem: "kinematics",
            track: track,
            columns: dofs,
            inDegrees: true,
            isMotionFile: true,
            radiansForRotations: radians,
            extract: { sample, name in
                guard let value = sample.jointAngles[name] else { return 0 }
                return isTranslationDOF(name) ? value : value * 180.0 / .pi
            }
        )
    }

    static func stoTextForInverseDynamics(track: OfflineAnalysisTrack) -> String {
        // Joint torques in Nm; OpenSim ID convention is name suffix _moment for
        // rotational and _force for translational, but we keep raw DOF names
        // for traceability against `track.dofNames`.
        return motOrStoText(
            stem: "inverse_dynamics",
            track: track,
            columns: track.dofNames,
            inDegrees: false,
            isMotionFile: false,
            radiansForRotations: true,
            extract: { sample, name in sample.jointTorques[name] ?? 0 }
        )
    }

    static func stoTextForActivations(track: OfflineAnalysisTrack) -> String {
        return motOrStoText(
            stem: "activations",
            track: track,
            columns: track.muscleNames,
            inDegrees: false,
            isMotionFile: false,
            radiansForRotations: true,
            extract: { sample, name in sample.activations[name] ?? 0 }
        )
    }

    static func stoTextForMuscleForces(track: OfflineAnalysisTrack) -> String {
        return motOrStoText(
            stem: "muscle_forces",
            track: track,
            columns: track.muscleNames,
            inDegrees: false,
            isMotionFile: false,
            radiansForRotations: true,
            extract: { sample, name in sample.muscleForces[name] ?? 0 }
        )
    }

    // MARK: - Summary JSON

    static func summary(for track: OfflineAnalysisTrack, stem: String) -> [String: Any] {
        let convergedCount = track.samples.filter(\.muscleConverged).count
        return [
            "stem": stem,
            "model": track.modelName ?? NSNull(),
            "motion": track.motionName ?? NSNull(),
            "frame_count": track.frameCount,
            "duration_s": track.duration,
            "fps": track.fps,
            "total_mass_kg": track.totalMassKg,
            "dof_count": track.dofNames.count,
            "muscle_count": track.muscleNames.count,
            "max_torque_per_kg_nm": track.maxTorquePerKg,
            "max_root_residual_per_kg_nm": track.maxRootResidualPerKg,
            "muscle_converged_frames": convergedCount,
            "muscle_converged_fraction": track.frameCount > 0
                ? Double(convergedCount) / Double(track.frameCount)
                : 0,
            "top_mean_activations": Array(
                track.meanActivationsByMuscle().prefix(10).map {
                    ["muscle": $0.name, "mean": $0.mean] as [String: Any]
                }
            ),
        ]
    }

    // MARK: - Internals

    private static func motOrStoText(
        stem: String,
        track: OfflineAnalysisTrack,
        columns: [String],
        inDegrees: Bool,
        isMotionFile: Bool,
        radiansForRotations: Bool,
        extract: (OfflineFrameSample, String) -> Double
    ) -> String {
        let startTime = track.samples.first?.timestamp ?? 0
        var lines: [String] = []
        lines.append(stem)
        lines.append("version=1")
        lines.append("nRows=\(track.samples.count)")
        lines.append("nColumns=\(columns.count + 1)")
        lines.append("inDegrees=\(inDegrees ? "yes" : "no")")
        lines.append("endheader")
        lines.append("time\t" + columns.joined(separator: "\t"))

        for sample in track.samples {
            let t = sample.timestamp - startTime
            var row = String(format: "%.6f", t)
            for name in columns {
                let value = extract(sample, name)
                row += String(format: "\t%.6f", value)
            }
            lines.append(row)
        }
        _ = isMotionFile  // currently identical layout for .mot vs .sto
        _ = radiansForRotations
        return lines.joined(separator: "\n") + "\n"
    }

    private static func isTranslationDOF(_ name: String) -> Bool {
        name.hasSuffix("_tx") || name.hasSuffix("_ty") || name.hasSuffix("_tz")
    }

    private static func makeOutputFolder(stem: String, parent: URL?) throws -> URL {
        // Write to Documents (not tmp) so files survive app restarts and are
        // visible in the Files app once UIFileSharingEnabled is set.
        let base: URL
        if let parent {
            base = parent
        } else {
            base = try FileManager.default.url(
                for: .documentDirectory,
                in: .userDomainMask,
                appropriateFor: nil,
                create: true
            )
        }
        let folder = base
            .appendingPathComponent("OfflineExport-\(stem)-\(UUID().uuidString.prefix(8))",
                                    isDirectory: true)
        try FileManager.default.createDirectory(at: folder, withIntermediateDirectories: true)
        return folder
    }

    private static func write(data text: String, to url: URL) throws {
        do {
            try text.write(to: url, atomically: true, encoding: .utf8)
        } catch {
            throw ExportError.writeFailed(url, underlying: error)
        }
    }

    private static func writeJSON(_ object: [String: Any], to url: URL) throws {
        do {
            let data = try JSONSerialization.data(
                withJSONObject: object,
                options: [.prettyPrinted, .sortedKeys]
            )
            try data.write(to: url, options: .atomic)
        } catch {
            throw ExportError.writeFailed(url, underlying: error)
        }
    }
}
