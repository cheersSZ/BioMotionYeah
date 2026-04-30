import Foundation
import Combine
import AVFoundation

@MainActor
final class OfflineSession: ObservableObject {
    @Published private(set) var nimble = NimbleEngine()
    @Published private(set) var modelURL: URL?
    @Published private(set) var motionURL: URL?
    @Published private(set) var videoURL: URL?
    @Published private(set) var videoPlayer: AVPlayer?
    @Published private(set) var importedMotion: ImportedMotionSeries?
    @Published private(set) var preparedMotion: ImportedMotionPreparation?
    @Published private(set) var currentFrameIndex = 0
    @Published private(set) var isPlaying = false
    @Published private(set) var keypointTrack: OfflineKeypointTrack?
    @Published private(set) var keypointMetadataURL: URL?
    @Published private(set) var currentKeypoints: [OfflineKeypoint] = []
    @Published private(set) var currentKeypointFrameIndex: Int = 0
    @Published var statusMessage = "Import an OpenSim .osim and OpenCap .mot file"
    @Published var warningMessage: String?
    @Published var errorMessage: String?
    @Published private(set) var lastSelectedImportName: String?

    /// Inverse-dynamics mode used by the next `processAllFrames()` run. Set
    /// from the UI before kicking off batch processing. Persisted into the
    /// resulting `OfflineAnalysisTrack.idMode` so re-loaded analyses know
    /// which path produced them.
    @Published var idMode: NimbleEngine.IDMode = .withGRF

    // Batch processing
    @Published private(set) var analysisTrack: OfflineAnalysisTrack?
    @Published private(set) var lastExportFolder: URL?
    @Published private(set) var isProcessingAll = false
    /// 0...1 progress while `processAllFrames` is running.
    @Published private(set) var processingProgress: Double = 0

    private var playbackTask: Task<Void, Never>?
    private var cancellables: Set<AnyCancellable> = []
    private var videoTimeObserver: Any?
    private var videoEndObserver: NSObjectProtocol?
    /// Becomes true once `warnIfTimelinesMismatch` actually had finite values
    /// for both video and motion durations to compare; resets on new import.
    private var timelineMismatchEvaluated = false

    init() {
        nimble.objectWillChange
            .receive(on: DispatchQueue.main)
            .sink { [weak self] _ in
                self?.objectWillChange.send()
            }
            .store(in: &cancellables)
    }

    /// Slider/playback timeline. Adapts to whichever data layer is current:
    /// after `processAllFrames` we drive the UI off `analysisTrack.samples`
    /// (the only frames with valid ID/SO results), otherwise off the raw
    /// imported motion frames so the user can still scrub video + keypoints.
    private var frameTimestamps: [TimeInterval] {
        if let track = analysisTrack {
            return track.samples.map(\.timestamp)
        }
        return preparedMotion?.frames.map(\.timestamp) ?? []
    }

    var frameCount: Int {
        frameTimestamps.count
    }

    var currentTimestamp: TimeInterval {
        let timestamps = frameTimestamps
        guard !timestamps.isEmpty else { return 0 }
        let idx = max(0, min(currentFrameIndex, timestamps.count - 1))
        return timestamps[idx]
    }

    /// The analyzed sample at `currentFrameIndex`, or `nil` if no batch run
    /// has produced a track yet (or the index is out of range during the
    /// brief window where `analysisTrack` is being swapped in).
    var currentAnalysisSample: OfflineFrameSample? {
        guard let track = analysisTrack,
              track.samples.indices.contains(currentFrameIndex) else { return nil }
        return track.samples[currentFrameIndex]
    }

    var duration: TimeInterval {
        if let videoDuration, videoDuration > 0 {
            return videoDuration
        }
        let timestamps = frameTimestamps
        guard let first = timestamps.first, let last = timestamps.last else { return 0 }
        return max(0, last - first)
    }

    var estimatedFPS: Double {
        let timestamps = frameTimestamps
        guard timestamps.count >= 2 else { return 0 }
        let dt = timestamps[1] - timestamps[0]
        guard dt > 0 else { return 0 }
        return 1.0 / dt
    }

    var playbackTimestamp: TimeInterval {
        if let videoPlayer {
            let time = videoPlayer.currentTime().seconds
            if time.isFinite {
                return max(0, time)
            }
        }
        return currentTimestamp
    }

    private var videoDuration: TimeInterval? {
        guard let seconds = videoPlayer?.currentItem?.duration.seconds, seconds.isFinite else {
            return nil
        }
        return max(0, seconds)
    }

    func importModel(from originalURL: URL) {
        stopPlayback()
        errorMessage = nil
        warningMessage = nil
        lastSelectedImportName = originalURL.lastPathComponent
        statusMessage = "Selected model: \(originalURL.lastPathComponent)"

        do {
            let sandboxURL = try copyImportedFileToSandbox(originalURL)
            statusMessage = "Loading model..."
            nimble.loadModel(fromPath: sandboxURL.path) { [weak self] success in
                guard let self else { return }
                if success {
                    self.modelURL = sandboxURL
                    self.statusMessage = "Model loaded: \(sandboxURL.lastPathComponent)"
                    self.prepareMotionIfPossible()
                } else {
                    self.errorMessage = "Failed to load model \(sandboxURL.lastPathComponent)"
                    self.statusMessage = "Model load failed"
                }
            }
        } catch {
            errorMessage = "Model import failed: \(error.localizedDescription)"
            statusMessage = "Model import failed"
        }
    }

    func importMotion(from originalURL: URL) {
        stopPlayback()
        errorMessage = nil
        warningMessage = nil
        timelineMismatchEvaluated = false
        lastSelectedImportName = originalURL.lastPathComponent
        statusMessage = "Selected motion: \(originalURL.lastPathComponent)"

        do {
            let sandboxURL = try copyImportedFileToSandbox(originalURL)
            let series = try MOTParser.parse(url: sandboxURL)
            motionURL = sandboxURL
            importedMotion = series
            statusMessage = "Motion loaded: \(sandboxURL.lastPathComponent)"
            prepareMotionIfPossible()
        } catch {
            errorMessage = "Motion import failed: \(error.localizedDescription)"
            statusMessage = "Motion import failed"
        }
    }

    /// Imports an offline keypoint pair (.keypoints.json + .keypoints.bin).
    /// `urls` may be in any order; the metadata JSON is detected by extension and the
    /// binary is matched either by being the second URL or by the filename declared
    /// inside the JSON (resolved relative to the chosen sandbox folder).
    func importKeypoints(from urls: [URL]) {
        errorMessage = nil
        warningMessage = nil
        guard !urls.isEmpty else { return }

        let firstName = urls.first?.lastPathComponent ?? ""
        lastSelectedImportName = firstName

        guard let metadataOriginal = urls.first(where: { $0.pathExtension.lowercased() == "json" }) else {
            errorMessage = "Select the .keypoints.json file (and optionally its .keypoints.bin pair)."
            statusMessage = "Keypoint import failed"
            return
        }
        let binaryOriginal = urls.first { $0 != metadataOriginal && $0.pathExtension.lowercased() == "bin" }

        do {
            let sandboxFolder = try makeSandboxFolder(prefix: "Keypoints")
            let metadataSandbox = sandboxFolder
                .appendingPathComponent(metadataOriginal.lastPathComponent)
            try copySecurityScopedItem(from: metadataOriginal, to: metadataSandbox)

            // Inspect metadata to learn the expected binary filename, then pull
            // the binary in alongside it (either the user's explicit pick or
            // the sibling matching the declared name).
            let metadata = try decodeMetadata(at: metadataSandbox)
            let binaryName = metadata.storage.binaryFilename
            let binarySandbox = sandboxFolder.appendingPathComponent(binaryName)

            if let binaryOriginal {
                try copySecurityScopedItem(from: binaryOriginal, to: binarySandbox)
            } else {
                // Fall back to the sibling next to the metadata file the user picked.
                let inferredOriginal = metadataOriginal
                    .deletingLastPathComponent()
                    .appendingPathComponent(binaryName)
                guard FileManager.default.isReadableFile(atPath: inferredOriginal.path) else {
                    throw OfflineKeypointTrack.LoadError.binaryNotFound(inferredOriginal)
                }
                try copySecurityScopedItem(from: inferredOriginal, to: binarySandbox)
            }

            let track = try OfflineKeypointTrack.load(metadataURL: metadataSandbox, binaryURL: binarySandbox)
            keypointTrack = track
            keypointMetadataURL = metadataSandbox
            statusMessage = "Keypoints loaded: \(track.frameCount) frames @ \(String(format: "%.1f", track.fps)) Hz"
            updateKeypointSample(time: playbackTimestamp)
        } catch {
            keypointTrack = nil
            keypointMetadataURL = nil
            currentKeypoints = []
            errorMessage = "Keypoint import failed: \(error.localizedDescription)"
            statusMessage = "Keypoint import failed"
        }
    }

    /// Imports a folder containing the full OpenCap dataset (model, motion,
    /// video, and keypoints). Scans the folder for the expected file types
    /// and chains the existing per-file import methods. Any missing files
    /// are reported via `errorMessage` and printed to the console; whatever
    /// is present is still loaded.
    func importOpenCapFolder(from folderURL: URL) {
        stopPlayback()
        errorMessage = nil
        warningMessage = nil
        timelineMismatchEvaluated = false
        lastSelectedImportName = folderURL.lastPathComponent
        statusMessage = "Scanning folder: \(folderURL.lastPathComponent)"

        let didAccess = folderURL.startAccessingSecurityScopedResource()
        defer {
            if didAccess { folderURL.stopAccessingSecurityScopedResource() }
        }

        let contents: [URL]
        do {
            contents = try FileManager.default.contentsOfDirectory(
                at: folderURL,
                includingPropertiesForKeys: nil,
                options: [.skipsHiddenFiles]
            )
        } catch {
            let message = "Cannot read folder \(folderURL.lastPathComponent): \(error.localizedDescription)"
            print("[OfflineSession] \(message)")
            errorMessage = message
            statusMessage = "Folder import failed"
            return
        }

        func first(withExtension ext: String) -> URL? {
            contents.first { $0.pathExtension.lowercased() == ext.lowercased() }
        }
        func first(withSuffix suffix: String) -> URL? {
            let needle = suffix.lowercased()
            return contents.first { $0.lastPathComponent.lowercased().hasSuffix(needle) }
        }

        let model = first(withExtension: "osim")
        let motion = first(withExtension: "mot")
        let video = first(withExtension: "mov") ?? first(withExtension: "mp4")
        let keypointsJSON = first(withSuffix: ".keypoints.json")
        let keypointsBin = first(withSuffix: ".keypoints.bin")

        var missing: [String] = []
        if model == nil { missing.append("*.osim") }
        if motion == nil { missing.append("*.mot") }
        if video == nil { missing.append("*.mov or *.mp4") }
        if keypointsJSON == nil { missing.append("*.keypoints.json") }
        if keypointsBin == nil { missing.append("*.keypoints.bin") }

        for name in missing {
            print("[OfflineSession] Missing in folder \"\(folderURL.lastPathComponent)\": \(name)")
        }

        // Load each present file. The per-file import methods clear their own
        // errorMessage at the start, so we collect missing-file errors and set
        // them once at the end (below) to avoid being clobbered.
        if let url = model { importModel(from: url) }
        if let url = motion { importMotion(from: url) }
        if let url = video { importVideo(from: url) }
        if let json = keypointsJSON, let bin = keypointsBin {
            importKeypoints(from: [json, bin])
        } else if let json = keypointsJSON {
            importKeypoints(from: [json])
        }

        if !missing.isEmpty {
            let combined = "Missing in \"\(folderURL.lastPathComponent)\": " + missing.joined(separator: ", ")
            if let existing = errorMessage, !existing.isEmpty {
                errorMessage = combined + "\n" + existing
            } else {
                errorMessage = combined
            }
            statusMessage = "Folder import: missing files (see error below)"
        } else if errorMessage == nil {
            statusMessage = "Loaded folder \(folderURL.lastPathComponent)"
        }
    }

    func importVideo(from originalURL: URL) {
        stopPlayback()
        errorMessage = nil
        warningMessage = nil
        timelineMismatchEvaluated = false
        lastSelectedImportName = originalURL.lastPathComponent
        statusMessage = "Selected video: \(originalURL.lastPathComponent)"

        do {
            let sandboxURL = try copyImportedFileToSandbox(originalURL)
            configureVideoPlayer(url: sandboxURL)
            videoURL = sandboxURL
            statusMessage = "Video loaded: \(sandboxURL.lastPathComponent)"
            syncToVideoTime()
            warnIfTimelinesMismatch()
        } catch {
            errorMessage = "Video import failed: \(error.localizedDescription)"
            statusMessage = "Video import failed"
        }
    }

    /// Move to a specific frame in the current timeline. Plan A: this is
    /// purely a navigation primitive — it never re-runs the IK/ID/SO solver.
    /// Per-frame analyzed data only comes from `processAllFrames`, because
    /// the SG filter inside `NimbleEngine` requires a sequential window of
    /// ~9 frames and gives garbage when fed a single random frame.
    func seek(to frameIndex: Int) {
        let timestamps = frameTimestamps
        guard !timestamps.isEmpty else { return }
        let clampedIndex = max(0, min(frameIndex, timestamps.count - 1))
        let timestamp = timestamps[clampedIndex]
        currentFrameIndex = clampedIndex
        if videoPlayer != nil {
            seekVideo(to: timestamp)
        }
        updateKeypointSample(time: timestamp)
    }

    /// Step `delta` frames forward (positive) or backward (negative). Used
    /// by the on-screen ◀ / ▶ buttons since the slider is hard to land on
    /// individual frames on a phone.
    func stepFrame(by delta: Int) {
        guard frameCount > 0, delta != 0 else { return }
        stopPlayback()
        seek(to: currentFrameIndex + delta)
    }

    func togglePlayback() {
        isPlaying ? stopPlayback() : startPlayback()
    }

    func resetToStart() {
        stopPlayback()
        seek(to: 0)
    }

    private func startPlayback() {
        if let videoPlayer {
            if let videoDuration, playbackTimestamp >= max(0, videoDuration - 0.01) {
                seekVideo(to: 0)
                currentFrameIndex = 0
            }
            isPlaying = true
            videoPlayer.play()
            return
        }
        let timestamps = frameTimestamps
        guard timestamps.count > 1 else { return }
        if currentFrameIndex >= timestamps.count - 1 {
            currentFrameIndex = 0
        }
        isPlaying = true
        playbackTask = Task { [weak self] in
            guard let self else { return }
            while !Task.isCancelled, self.currentFrameIndex < timestamps.count - 1 {
                let nextIndex = self.currentFrameIndex + 1
                let currentTime = timestamps[self.currentFrameIndex]
                let nextTime = timestamps[nextIndex]
                let delayNs = UInt64(max(0.001, min(nextTime - currentTime, 1.0 / 30.0)) * 1_000_000_000)
                try? await Task.sleep(nanoseconds: delayNs)
                if Task.isCancelled { break }
                self.seek(to: nextIndex)
            }
            self.isPlaying = false
            self.playbackTask = nil
        }
    }

    private func stopPlayback() {
        videoPlayer?.pause()
        playbackTask?.cancel()
        playbackTask = nil
        isPlaying = false
    }

    /// Run IK→ID→SO over every prepared frame in order and capture the result
    /// into `analysisTrack`. Idempotent: replaces any prior track. Returns
    /// without doing anything if no motion is loaded or another batch run is
    /// already in flight.
    ///
    /// Note: the SG filter inside `NimbleEngine` warms up over the first ~9
    /// frames and emits center-timestamped output 4 frames behind the input,
    /// so the captured track is shorter than `preparedMotion.frames` by ~8.
    func processAllFrames() {
        guard !isProcessingAll else { return }
        guard let prepared = preparedMotion, !prepared.frames.isEmpty else {
            warningMessage = "Load a model and motion before running batch analysis."
            return
        }

        stopPlayback()
        analysisTrack = nil
        lastExportFolder = nil
        isProcessingAll = true
        processingProgress = 0
        statusMessage = "Processing \(prepared.frames.count) frames…"

        let totalFrames = prepared.frames.count
        let frames = prepared.frames
        let dofNames = prepared.modelDOFNames
        let modelName = modelURL?.lastPathComponent
        let motionName = motionURL?.lastPathComponent

        // Lock the ID path for this batch run before any frames dispatch.
        // `processImportedMotionFrame` reads `nimble.idMode` per-frame, so
        // setting it here guarantees every frame uses the same mode and the
        // resulting track tags consistently below in `finishBatchRun`.
        nimble.idMode = idMode
        nimble.resetMotionFilters()
        nimble.startRecordingResults()

        Task.detached(priority: .userInitiated) { [weak self] in
            guard let self else { return }
            // Dispatch frames one at a time onto the engine's serial solver
            // queue and update progress from the main actor periodically.
            // Drains `solverQueue` between progress ticks so the UI doesn't
            // sit at 0% until the whole batch is done.
            let progressStride = max(1, totalFrames / 100)
            for (i, frame) in frames.enumerated() {
                await MainActor.run { self.nimble.processImportedMotionFrame(frame) }
                if (i + 1) % progressStride == 0 || i == totalFrames - 1 {
                    await MainActor.run { self.nimble.waitForPendingSolverWork() }
                    let p = Double(i + 1) / Double(totalFrames)
                    await MainActor.run {
                        self.processingProgress = p
                    }
                }
            }
            await MainActor.run { self.nimble.waitForPendingSolverWork() }
            await MainActor.run { self.finishBatchRun(dofNames: dofNames,
                                                     modelName: modelName,
                                                     motionName: motionName) }
        }
    }

    /// Export the most recent batch result to the temp directory. Returns the
    /// produced bundle of files, or throws if no track is present.
    func exportAnalysis() throws -> OfflineAnalysisExporter.Output {
        guard let track = analysisTrack else {
            throw OfflineAnalysisExporter.ExportError.noFrames
        }
        let stem = (motionURL?.deletingPathExtension().lastPathComponent ?? "offline")
        let output = try OfflineAnalysisExporter.export(track: track, stem: stem)
        lastExportFolder = output.folder
        statusMessage = "Exported to \(output.folder.lastPathComponent)"
        return output
    }

    /// Rehydrate a previously-exported analysis folder so the
    /// analysis-track-driven UI (badges, IK readout, muscle bar, slider)
    /// works without re-running IK/ID/SO. Use this after a fresh install
    /// to skip a multi-second processing pass.
    ///
    /// `folderURL` is the export folder produced by `exportAnalysis()`
    /// (the one whose name starts with `OfflineExport-`). We scan it for a
    /// single `*_processed.json` blob — the lossless round-trip file —
    /// and ignore the OpenSim `.mot` / `.sto` siblings, which are for
    /// external tools.
    ///
    /// The processed blob is self-contained: it carries DOF names, muscle
    /// names, mass, and every per-frame sample, so neither the .osim model
    /// nor the .mot motion needs to be loaded first. A separately-imported
    /// video (and keypoints, if any) is preserved so the user can scrub
    /// the cached analysis against the original footage.
    func loadProcessedAnalysisFolder(from folderURL: URL) {
        stopPlayback()
        errorMessage = nil
        warningMessage = nil
        lastSelectedImportName = folderURL.lastPathComponent
        statusMessage = "Scanning folder: \(folderURL.lastPathComponent)"

        let didAccess = folderURL.startAccessingSecurityScopedResource()
        defer {
            if didAccess { folderURL.stopAccessingSecurityScopedResource() }
        }

        let processedOriginal: URL
        do {
            processedOriginal = try findProcessedJSON(in: folderURL)
        } catch {
            errorMessage = "Processed analysis import failed: \(error.localizedDescription)"
            statusMessage = "Processed analysis import failed"
            return
        }

        do {
            let sandboxURL = try copyImportedFileToSandbox(processedOriginal)
            let track = try OfflineAnalysisExporter.loadProcessed(from: sandboxURL)
            analysisTrack = track
            lastExportFolder = nil
            currentFrameIndex = 0

            if track.samples.isEmpty {
                statusMessage = "Loaded processed analysis but it contains no frames."
                warningMessage = "The processed file is empty — re-export from a non-empty run."
                return
            }

            // Re-bind the playhead/video to the cached timeline. If a video
            // was imported separately for the same trial, this snaps it to
            // frame 0 of the cached analysis so the badges line up.
            seek(to: 0)
            statusMessage = "Loaded \(track.frameCount) processed frames @ \(String(format: "%.1f", track.fps)) Hz from \(processedOriginal.lastPathComponent)"
        } catch {
            analysisTrack = nil
            errorMessage = "Processed analysis import failed: \(error.localizedDescription)"
            statusMessage = "Processed analysis import failed"
        }
    }

    enum ProcessedFolderError: Error, LocalizedError {
        case folderUnreadable(URL, underlying: Error)
        case noProcessedJSON(URL, sawNames: [String])
        case ambiguousProcessedJSON(URL, candidates: [String])
        case iCloudNotDownloaded(URL, placeholderNames: [String])

        var errorDescription: String? {
            switch self {
            case .folderUnreadable(let url, let error):
                return "Cannot read folder \(url.lastPathComponent): \(error.localizedDescription)"
            case .noProcessedJSON(let url, let sawNames):
                let preview = sawNames.isEmpty
                    ? "(folder is empty)"
                    : sawNames.prefix(8).joined(separator: ", ")
                    + (sawNames.count > 8 ? ", …" : "")
                return "No *_processed.json found in \(url.lastPathComponent). "
                    + "Saw: \(preview). "
                    + "Pick the OfflineExport-… folder produced by Export "
                    + "(or its parent containing exactly one such folder)."
            case .ambiguousProcessedJSON(let url, let candidates):
                return "Multiple *_processed.json files in \(url.lastPathComponent): \(candidates.joined(separator: ", ")). Pick a folder with exactly one."
            case .iCloudNotDownloaded(let url, let placeholderNames):
                let preview = placeholderNames.prefix(4).joined(separator: ", ")
                return "Found iCloud placeholder(s) in \(url.lastPathComponent) "
                    + "but the file isn't downloaded yet: \(preview). "
                    + "Open the folder in the Files app, tap the cloud-download "
                    + "icon next to the *_processed.json file, then try again."
            }
        }
    }

    /// Locate the lossless `*_processed.json` blob in a user-picked folder.
    ///
    /// Robust to three real-world failure modes we kept hitting on iOS:
    /// 1. **iCloud placeholders** — undownloaded files appear as hidden
    ///    `.<name>_processed.json.icloud`. We detect them, kick off a
    ///    download, and surface a clear "tap to download in Files" error
    ///    instead of pretending the file doesn't exist.
    /// 2. **Wrong folder picked** — if the user picks the parent that
    ///    *contains* a single `OfflineExport-…` subfolder, we descend into
    ///    that subfolder rather than failing.
    /// 3. **Mystery contents** — the error message lists the actual file
    ///    names we saw, so the user can tell at a glance whether they
    ///    picked the right folder.
    private func findProcessedJSON(in folderURL: URL) throws -> URL {
        switch try scanFolderForProcessedJSON(folderURL) {
        case .found(let url):
            return url
        case .placeholders(let names):
            throw ProcessedFolderError.iCloudNotDownloaded(folderURL, placeholderNames: names)
        case .ambiguous(let names):
            throw ProcessedFolderError.ambiguousProcessedJSON(folderURL, candidates: names)
        case .none(let topNames):
            // Fallback: descend one level. Picking the parent of an
            // OfflineExport-… folder is a common mistake on iOS where the
            // file picker collapses single-folder parents into themselves.
            let subfolders = try directChildSubfolders(of: folderURL)
            for sub in subfolders {
                if case .found(let url) = try scanFolderForProcessedJSON(sub) {
                    return url
                }
            }
            throw ProcessedFolderError.noProcessedJSON(folderURL, sawNames: topNames)
        }
    }

    private enum ProcessedScan {
        case found(URL)
        case placeholders([String])
        case ambiguous([String])
        case none([String])
    }

    private func scanFolderForProcessedJSON(_ folderURL: URL) throws -> ProcessedScan {
        let contents: [URL]
        do {
            contents = try FileManager.default.contentsOfDirectory(
                at: folderURL,
                includingPropertiesForKeys: [.isRegularFileKey],
                // No .skipsHiddenFiles — iCloud placeholders are hidden by
                // convention and we want to surface them as a clear error.
                options: []
            )
        } catch {
            throw ProcessedFolderError.folderUnreadable(folderURL, underlying: error)
        }

        let visibleNames = contents
            .map(\.lastPathComponent)
            .filter { !$0.hasPrefix(".") }
            .sorted()

        let matches = contents.filter {
            $0.lastPathComponent.lowercased().hasSuffix("_processed.json")
        }
        if matches.count == 1 {
            return .found(matches[0])
        }
        if matches.count > 1 {
            return .ambiguous(matches.map(\.lastPathComponent).sorted())
        }

        // No materialized match — look for iCloud placeholders matching
        // `.<stem>_processed.json.icloud`. If we find one, request a
        // download so the next attempt has a real file to read.
        let placeholders = contents.filter { url in
            let name = url.lastPathComponent
            return name.hasPrefix(".")
                && name.lowercased().hasSuffix("_processed.json.icloud")
        }
        if !placeholders.isEmpty {
            for ph in placeholders {
                try? FileManager.default.startDownloadingUbiquitousItem(at: ph)
            }
            let realNames = placeholders.map(stripICloudPlaceholder)
            return .placeholders(realNames)
        }

        return .none(visibleNames)
    }

    private func directChildSubfolders(of folderURL: URL) throws -> [URL] {
        let contents: [URL]
        do {
            contents = try FileManager.default.contentsOfDirectory(
                at: folderURL,
                includingPropertiesForKeys: [.isDirectoryKey],
                options: [.skipsHiddenFiles]
            )
        } catch {
            throw ProcessedFolderError.folderUnreadable(folderURL, underlying: error)
        }
        return contents.filter { url in
            (try? url.resourceValues(forKeys: [.isDirectoryKey]).isDirectory) == true
        }
    }

    private func stripICloudPlaceholder(_ url: URL) -> String {
        var name = url.lastPathComponent
        if name.hasPrefix(".") { name.removeFirst() }
        if name.lowercased().hasSuffix(".icloud") {
            name.removeLast(".icloud".count)
        }
        return name
    }

    private func finishBatchRun(dofNames: [String], modelName: String?, motionName: String?) {
        nimble.stopRecordingResults()
        let track = OfflineAnalysisTrack.capture(
            from: nimble,
            dofNames: dofNames,
            modelName: modelName,
            motionName: motionName,
            idMode: nimble.idMode
        )
        analysisTrack = track
        processingProgress = 1
        isProcessingAll = false
        // After the batch run, the slider re-binds to `analysisTrack.samples`
        // (shorter than the imported motion by ~8 frames due to SG warm-up).
        // Re-snap the playhead onto a valid sample so the inspector + video
        // line up with what the user was looking at before the run.
        if track.samples.isEmpty {
            currentFrameIndex = 0
            statusMessage = "Batch run produced no frames — SG filter may not have warmed up. Need at least 9 input frames."
        } else {
            currentFrameIndex = min(currentFrameIndex, track.samples.count - 1)
            seek(to: currentFrameIndex)
            statusMessage = "Processed \(track.frameCount) frames @ \(String(format: "%.1f", track.fps)) Hz"
        }
    }

    private func prepareMotionIfPossible() {
        guard nimble.isModelLoaded else {
            warningMessage = "Load an OpenSim model before preparing motion."
            preparedMotion = nil
            return
        }
        guard let importedMotion else { return }
        let prepared = nimble.prepareImportedMotion(importedMotion)
        preparedMotion = prepared
        nimble.resetMotionFilters()
        // Drop any stale analysis track from a previous motion/model so the
        // slider rebinds to the new motion's raw frames until the user runs
        // a fresh batch analysis.
        analysisTrack = nil
        lastExportFolder = nil
        currentFrameIndex = 0

        guard let prepared else {
            warningMessage = "Motion preparation failed."
            statusMessage = "Motion preparation failed"
            return
        }

        var warnings: [String] = []
        if !prepared.unmatchedSourceCoordinates.isEmpty {
            warnings.append("Unmatched source coordinates: \(prepared.unmatchedSourceCoordinates.prefix(8).joined(separator: ", "))")
        }
        if !prepared.unmatchedModelDOFs.isEmpty {
            warnings.append("Model DOFs without source data: \(prepared.unmatchedModelDOFs.prefix(8).joined(separator: ", "))")
        }
        warningMessage = warnings.isEmpty ? nil : warnings.joined(separator: "\n")
        statusMessage = "Prepared \(prepared.frames.count) frames for offline playback"

        if videoPlayer != nil {
            syncToVideoTime()
        } else if !prepared.frames.isEmpty {
            seek(to: 0)
        }

        warnIfTimelinesMismatch()
    }

    /// Flag a noticeable mismatch between the imported motion duration and
    /// the loaded video duration. We tolerate small differences because the
    /// trail2 dataset itself shows ~0.15 s drift between `.mot` and `.mov`.
    /// Threshold of 1.0 s catches "wrong file" mistakes without false-positiving
    /// frame-count rounding differences.
    private func warnIfTimelinesMismatch() {
        guard !timelineMismatchEvaluated,
              let videoDuration,
              let prepared = preparedMotion,
              let first = prepared.frames.first,
              let last = prepared.frames.last else {
            return
        }
        timelineMismatchEvaluated = true
        let motDuration = max(0, last.timestamp - first.timestamp)
        let delta = abs(motDuration - videoDuration)
        guard delta > 1.0 else { return }
        let extra = "Motion is \(String(format: "%.2fs", motDuration)); video is \(String(format: "%.2fs", videoDuration)). They likely come from different trials."
        if let existing = warningMessage, !existing.isEmpty {
            warningMessage = existing + "\n" + extra
        } else {
            warningMessage = extra
        }
    }

    private func configureVideoPlayer(url: URL) {
        if let observer = videoTimeObserver, let videoPlayer {
            videoPlayer.removeTimeObserver(observer)
        }
        if let videoEndObserver {
            NotificationCenter.default.removeObserver(videoEndObserver)
        }

        let playerItem = AVPlayerItem(url: url)
        let player = AVPlayer(playerItem: playerItem)
        player.actionAtItemEnd = .pause

        let interval = CMTime(seconds: 1.0 / 30.0, preferredTimescale: 600)
        videoTimeObserver = player.addPeriodicTimeObserver(forInterval: interval, queue: .main) { [weak self] _ in
            Task { @MainActor [weak self] in
                self?.syncToVideoTime()
            }
        }
        videoEndObserver = NotificationCenter.default.addObserver(
            forName: .AVPlayerItemDidPlayToEndTime,
            object: playerItem,
            queue: .main
        ) { [weak self] _ in
            Task { @MainActor [weak self] in
                self?.isPlaying = false
                self?.syncToVideoTime()
            }
        }

        videoPlayer = player
    }

    private func syncToVideoTime() {
        let timestamp = playbackTimestamp
        let timestamps = frameTimestamps
        if !timestamps.isEmpty {
            let index = nearestIndex(for: timestamp, in: timestamps)
            currentFrameIndex = index
        }
        updateKeypointSample(time: timestamp)
        // AVPlayerItem.duration is loaded asynchronously, so the first call
        // immediately after import often sees a NaN duration. Re-evaluate on
        // the periodic time observer until both durations are finite once.
        warnIfTimelinesMismatch()
    }

    private func seekVideo(to timestamp: TimeInterval) {
        guard let videoPlayer else { return }
        let clampedTime = max(0, timestamp)
        let target = CMTime(seconds: clampedTime, preferredTimescale: 600)
        videoPlayer.seek(to: target, toleranceBefore: .zero, toleranceAfter: .zero)
    }

    private func updateKeypointSample(time: TimeInterval) {
        guard let track = keypointTrack else {
            if !currentKeypoints.isEmpty {
                currentKeypoints = []
                currentKeypointFrameIndex = 0
            }
            return
        }
        let index = track.frameIndex(for: time)
        if index == currentKeypointFrameIndex && !currentKeypoints.isEmpty { return }
        if let frame = track.frame(at: index) {
            currentKeypointFrameIndex = index
            currentKeypoints = frame
        }
    }

    private func decodeMetadata(at url: URL) throws -> OfflineKeypointMetadata {
        let data = try Data(contentsOf: url)
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        return try decoder.decode(OfflineKeypointMetadata.self, from: data)
    }

    private func makeSandboxFolder(prefix: String) throws -> URL {
        let folder = FileManager.default.temporaryDirectory
            .appendingPathComponent("\(prefix)-\(UUID().uuidString)", isDirectory: true)
        try FileManager.default.createDirectory(at: folder, withIntermediateDirectories: true)
        return folder
    }

    private func copySecurityScopedItem(from source: URL, to destination: URL) throws {
        let didAccess = source.startAccessingSecurityScopedResource()
        defer {
            if didAccess { source.stopAccessingSecurityScopedResource() }
        }
        if FileManager.default.fileExists(atPath: destination.path) {
            try FileManager.default.removeItem(at: destination)
        }
        try FileManager.default.copyItem(at: source, to: destination)
    }

    /// Binary search for the index whose timestamp is nearest `timestamp`.
    /// Generic over a sorted-ascending `[TimeInterval]` so it works for both
    /// raw motion frames and analysis-track samples.
    private func nearestIndex(for timestamp: TimeInterval, in timestamps: [TimeInterval]) -> Int {
        guard !timestamps.isEmpty else { return 0 }
        if timestamp <= timestamps[0] {
            return 0
        }
        let lastIndex = timestamps.count - 1
        if timestamp >= timestamps[lastIndex] {
            return lastIndex
        }

        var low = 0
        var high = lastIndex
        while low < high {
            let mid = (low + high) / 2
            if timestamps[mid] < timestamp {
                low = mid + 1
            } else {
                high = mid
            }
        }

        let upperIndex = low
        let lowerIndex = max(0, upperIndex - 1)
        let lowerDelta = abs(timestamps[lowerIndex] - timestamp)
        let upperDelta = abs(timestamps[upperIndex] - timestamp)
        return lowerDelta <= upperDelta ? lowerIndex : upperIndex
    }

    private func copyImportedFileToSandbox(_ url: URL) throws -> URL {
        let didAccess = url.startAccessingSecurityScopedResource()
        defer {
            if didAccess {
                url.stopAccessingSecurityScopedResource()
            }
        }

        let destinationDirectory = FileManager.default.temporaryDirectory.appendingPathComponent(
            "OfflineImports",
            isDirectory: true
        )
        try FileManager.default.createDirectory(at: destinationDirectory, withIntermediateDirectories: true)

        let destinationURL = destinationDirectory.appendingPathComponent(UUID().uuidString)
            .appendingPathExtension(url.pathExtension)
        if FileManager.default.fileExists(atPath: destinationURL.path) {
            try FileManager.default.removeItem(at: destinationURL)
        }
        try FileManager.default.copyItem(at: url, to: destinationURL)
        return destinationURL
    }
}

private extension Array {
    subscript(safe index: Int) -> Element? {
        guard indices.contains(index) else { return nil }
        return self[index]
    }
}
