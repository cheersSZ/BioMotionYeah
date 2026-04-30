import SwiftUI
import UniformTypeIdentifiers
import AVKit
import AVFoundation

struct OfflineAnalysisView: View {
    @StateObject private var session = OfflineSession()
    @State private var isShowingFolderImporter = false
    @State private var isShowingProcessedImporter = false
    @State private var exportShareItems: [URL] = []
    @State private var isShowingExportShare = false

    @State private var showOverlays = true
    @State private var isShowingSourceSheet = false
    @State private var isShowingBatchSheet = false
    @AppStorage("offline.hasSeenSourceSheet") private var hasSeenSourceSheet = false

    var body: some View {
        NavigationStack {
            GeometryReader { geo in
                Group {
                    if geo.size.height < 600 {
                        ScrollView { mainLayout }
                    } else {
                        mainLayout
                    }
                }
                .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .top)
            }
            .navigationTitle("Offline")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .topBarTrailing) {
                    Button {
                        isShowingSourceSheet = true
                    } label: {
                        Image(systemName: "tray.and.arrow.down")
                    }
                    .accessibilityLabel("Source files")
                }
                ToolbarItem(placement: .topBarTrailing) {
                    Button {
                        isShowingBatchSheet = true
                    } label: {
                        Image(systemName: "square.stack.3d.up")
                    }
                    .accessibilityLabel("Batch analysis")
                }
            }
            .sheet(isPresented: $isShowingSourceSheet) {
                NavigationStack { sourceSheetContent }
                    .presentationDetents([.medium, .large])
            }
            .sheet(isPresented: $isShowingBatchSheet) {
                NavigationStack { batchSheetContent }
                    .presentationDetents([.medium, .large])
            }
            .alert("Offline Import Error", isPresented: errorBinding) {
                Button("OK") { session.errorMessage = nil }
            } message: {
                Text(session.errorMessage ?? "Unknown error")
            }
            .onAppear { autoOpenSourceIfFirstLaunch() }
        }
    }

    /// Compact phone layout: video card with floating analysis overlays,
    /// transport row, then a 2x3 badge grid. Sized to fit on iPhone 14/15/16
    /// without scrolling; the GeometryReader gate falls back to a ScrollView
    /// on devices shorter than 600pt of content height.
    private var mainLayout: some View {
        VStack(alignment: .leading, spacing: 12) {
            videoCard

            if session.videoPlayer != nil || session.frameCount > 0 {
                playbackTransport
                badgeGrid
            } else {
                emptyImportPrompt
            }
        }
        .padding(16)
    }

    // MARK: - Video card with floating overlays

    private var videoCard: some View {
        ZStack(alignment: .topTrailing) {
            videoBase
                .overlay(alignment: .topLeading) {
                    if let sample = session.currentAnalysisSample {
                        IKReadoutPanel(
                            ikResult: NimbleEngine.IKOutput(
                                jointAngles: sample.jointAngles,
                                error: sample.ikError,
                                timestamp: sample.timestamp
                            ),
                            idResult: makeIDOutput(from: sample)
                        )
                        .padding(8)
                        .opacity(showOverlays ? 1 : 0)
                        .animation(.easeInOut(duration: 0.2), value: showOverlays)
                        // Only intercept gestures while visible so the
                        // panel's internal horizontal ScrollView is
                        // scrollable; faded-out overlay must not block
                        // anything underneath.
                        .allowsHitTesting(showOverlays)
                    }
                }
                .overlay(alignment: .bottomLeading) {
                    if let sample = session.currentAnalysisSample {
                        MuscleActivationBar(muscle: NimbleEngine.MuscleOutput(
                            activations: sample.activations,
                            forces: sample.muscleForces,
                            converged: sample.muscleConverged,
                            timestamp: sample.timestamp
                        ))
                        .padding(8)
                        .opacity(showOverlays ? 1 : 0)
                        .animation(.easeInOut(duration: 0.2), value: showOverlays)
                        .allowsHitTesting(showOverlays)
                    }
                }

            if session.currentAnalysisSample != nil {
                overlayToggle
                    .padding(8)
            }
        }
    }

    @ViewBuilder
    private var videoBase: some View {
        if let videoPlayer = session.videoPlayer {
            ZStack {
                PlayerLayerView(player: videoPlayer)
                if let track = session.keypointTrack, !session.currentKeypoints.isEmpty {
                    PoseOverlayView(
                        keypoints: session.currentKeypoints,
                        pixelSize: CGSize(width: track.pixelWidth, height: track.pixelHeight)
                    )
                }
            }
            .frame(height: 240)
            .clipShape(RoundedRectangle(cornerRadius: 12))
        } else if let track = session.keypointTrack, !session.currentKeypoints.isEmpty {
            ZStack {
                Color.black
                PoseOverlayView(
                    keypoints: session.currentKeypoints,
                    pixelSize: CGSize(width: track.pixelWidth, height: track.pixelHeight)
                )
            }
            .frame(height: 240)
            .clipShape(RoundedRectangle(cornerRadius: 12))
        } else {
            ZStack {
                Color.black.opacity(0.85)
                Text("No video loaded")
                    .font(.subheadline)
                    .foregroundStyle(.white.opacity(0.7))
            }
            .frame(height: 240)
            .clipShape(RoundedRectangle(cornerRadius: 12))
        }
    }

    private var overlayToggle: some View {
        Button {
            showOverlays.toggle()
        } label: {
            Image(systemName: showOverlays ? "eye.slash" : "eye")
                .font(.callout)
                .foregroundStyle(.white)
                .padding(8)
                .background(.black.opacity(0.5), in: Circle())
        }
        .accessibilityLabel(showOverlays ? "Hide overlays" : "Show overlays")
    }

    // MARK: - Transport (slider + frame counter + ◀ ⏸ ▶▶ Reset)

    private var playbackTransport: some View {
        VStack(spacing: 8) {
            if session.frameCount > 0 {
                Slider(
                    value: Binding(
                        get: { Double(session.currentFrameIndex) },
                        set: { session.seek(to: Int($0.rounded())) }
                    ),
                    in: 0...Double(max(0, session.frameCount - 1)),
                    step: 1
                )

                HStack {
                    Text("Frame \(session.currentFrameIndex + 1) / \(session.frameCount)")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                    Spacer()
                    Text(String(format: "%.3fs", session.playbackTimestamp))
                        .font(.caption.monospacedDigit())
                        .foregroundStyle(.secondary)
                }
            } else if session.videoPlayer != nil {
                HStack {
                    Text("No motion loaded — video playback only.")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                    Spacer()
                    Text(String(format: "%.3fs", session.playbackTimestamp))
                        .font(.caption.monospacedDigit())
                        .foregroundStyle(.secondary)
                }
            }

            playbackControls
        }
    }

    /// Playback transport: prev / play / next with finger-friendly hit areas
    /// (the slider alone is too thin to land on individual frames on a phone).
    private var playbackControls: some View {
        HStack(spacing: 12) {
            transportButton(systemName: "backward.frame.fill", label: "Previous frame") {
                session.stepFrame(by: -1)
            }
            .disabled(session.frameCount == 0 || session.currentFrameIndex <= 0)

            Button(action: { session.togglePlayback() }) {
                Image(systemName: session.isPlaying ? "pause.fill" : "play.fill")
                    .font(.title3)
                    .frame(width: 44, height: 44)
            }
            .buttonStyle(.borderedProminent)
            .accessibilityLabel(session.isPlaying ? "Pause" : "Play")

            transportButton(systemName: "forward.frame.fill", label: "Next frame") {
                session.stepFrame(by: +1)
            }
            .disabled(session.frameCount == 0 || session.currentFrameIndex >= session.frameCount - 1)

            Spacer()

            Button("Reset") {
                session.resetToStart()
            }
            .buttonStyle(.bordered)
        }
    }

    private func transportButton(systemName: String, label: String, action: @escaping () -> Void) -> some View {
        Button(action: action) {
            Image(systemName: systemName)
                .font(.title3)
                .frame(width: 44, height: 44)
        }
        .buttonStyle(.bordered)
        .accessibilityLabel(label)
    }

    // MARK: - Compact 2x3 analysis badge grid
    //
    // Reads ID/SO/EMG straight out of `analysisTrack.samples` (computed
    // sequentially by `processAllFrames`, so derivatives and SG warm-up are
    // valid). Seeking with the slider or ◀/▶ buttons just changes which sample
    // we render — nothing is re-solved per frame.

    private var badgeGrid: some View {
        Group {
            if let sample = session.currentAnalysisSample, let track = session.analysisTrack {
                let mass = track.totalMassKg
                // SwiftUI Grid keeps column widths in sync across rows;
                // every cell uses `.frame(maxWidth: .infinity)` so the
                // columns are equal-width as well.
                Grid(horizontalSpacing: 8, verticalSpacing: 8) {
                    GridRow {
                        summaryBadge("SO", value: sample.muscleConverged ? "OK" : "FAIL")
                        summaryBadge("|τ|", value: String(format: "%.1f Nm", sample.maxAbsTorqueNm))
                        summaryBadge("|τ|/m", value: massNormalized(sample.maxAbsTorqueNm, mass: mass))
                    }
                    GridRow {
                        summaryBadge("Root Res/m", value: massNormalized(sample.rootResidualNorm, mass: mass))
                        summaryBadge("L Foot", value: footLabel(force: sample.leftFootForce, contact: sample.leftFootInContact))
                        summaryBadge("R Foot", value: footLabel(force: sample.rightFootForce, contact: sample.rightFootInContact))
                    }
                }
            } else {
                badgeGridPlaceholder
            }
        }
    }

    /// Holds the same vertical footprint as a populated 2x3 badge grid
    /// (~92pt: two ~38pt badges + 8pt spacing + a few pts for inset) so the
    /// surrounding layout doesn't shift after Process All Frames completes.
    private var badgeGridPlaceholder: some View {
        VStack {
            Text(badgeGridEmptyMessage)
                .font(.caption)
                .foregroundStyle(.secondary)
                .multilineTextAlignment(.center)
                .frame(maxWidth: .infinity)
                .padding(12)
        }
        .frame(minHeight: 92)
        .background(Color(.secondarySystemBackground), in: RoundedRectangle(cornerRadius: 12))
    }

    private var badgeGridEmptyMessage: String {
        if session.frameCount == 0 {
            return "Open Source ▾ to import a model + motion + video, or a video alone."
        }
        return "Tap Batch ▸ Process All Frames to compute IK→ID→SO. Per-frame inspection needs the full sequential run."
    }

    // MARK: - Empty state when nothing is loaded

    private var emptyImportPrompt: some View {
        VStack(spacing: 8) {
            Image(systemName: "tray.and.arrow.down")
                .font(.largeTitle)
                .foregroundStyle(.secondary)
            Text("Import a video, or a model + motion file, to enable offline playback.")
                .font(.subheadline)
                .foregroundStyle(.secondary)
                .multilineTextAlignment(.center)
            Button("Open Source") {
                isShowingSourceSheet = true
            }
            .buttonStyle(.borderedProminent)
            .padding(.top, 4)
        }
        .frame(maxWidth: .infinity)
        .padding(20)
        .background(.thinMaterial, in: RoundedRectangle(cornerRadius: 16))
    }

    // MARK: - Source sheet (importer + file metadata + prepared motion summary)

    private var sourceSheetContent: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 16) {
                importSection
                summarySection
            }
            .padding(16)
        }
        .navigationTitle("Source")
        .navigationBarTitleDisplayMode(.inline)
        .toolbar {
            ToolbarItem(placement: .topBarTrailing) {
                Button("Done") { isShowingSourceSheet = false }
            }
        }
        .fileImporter(
            isPresented: $isShowingFolderImporter,
            allowedContentTypes: [.folder],
            allowsMultipleSelection: false
        ) { result in
            handleFolderImport(result)
        }
    }

    private var importSection: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("OpenCap Import")
                .font(.headline)

            VStack(alignment: .leading, spacing: 8) {
                Button("Import OpenCap Folder…") {
                    session.statusMessage = "Choose a folder containing .osim, .mot, .mov, .keypoints.json, .keypoints.bin"
                    isShowingFolderImporter = true
                }
                .buttonStyle(.borderedProminent)

                Text("Folder should contain: .osim, .mot, .mov (or .mp4), .keypoints.json, .keypoints.bin")
                    .font(.caption2)
                    .foregroundStyle(.secondary)
            }

            if let modelURL = session.modelURL {
                labeledValue("Model", value: modelURL.lastPathComponent)
            }
            if let motionURL = session.motionURL {
                labeledValue("Motion", value: motionURL.lastPathComponent)
            }
            if let videoURL = session.videoURL {
                labeledValue("Video", value: videoURL.lastPathComponent)
            }
            if let keypointURL = session.keypointMetadataURL,
               let track = session.keypointTrack {
                labeledValue(
                    "Keys",
                    value: "\(keypointURL.lastPathComponent) — \(track.frameCount) frames @ \(String(format: "%.1f", track.fps)) Hz"
                )
            }

            Text(session.statusMessage)
                .font(.subheadline)
                .foregroundStyle(.secondary)

            if let selectedName = session.lastSelectedImportName {
                labeledValue("Selected", value: selectedName)
            }

            if let warningMessage = session.warningMessage {
                Text(warningMessage)
                    .font(.caption)
                    .foregroundStyle(.orange)
            }

            if let errorMessage = session.errorMessage {
                Text(errorMessage)
                    .font(.caption)
                    .foregroundStyle(.red)
            }
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(16)
        .background(.thinMaterial, in: RoundedRectangle(cornerRadius: 16))
    }

    private var summarySection: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("Prepared Motion")
                .font(.headline)

            HStack {
                summaryBadge("Frames", value: "\(session.frameCount)")
                summaryBadge("Duration", value: String(format: "%.2fs", session.duration))
                summaryBadge("FPS", value: session.estimatedFPS > 0 ? String(format: "%.1f", session.estimatedFPS) : "—")
            }
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(16)
        .background(.thinMaterial, in: RoundedRectangle(cornerRadius: 16))
    }

    // MARK: - Batch sheet (Process All Frames + Export)

    private var batchSheetContent: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 16) {
                batchSection
            }
            .padding(16)
        }
        .navigationTitle("Batch")
        .navigationBarTitleDisplayMode(.inline)
        .toolbar {
            ToolbarItem(placement: .topBarTrailing) {
                Button("Done") { isShowingBatchSheet = false }
            }
        }
        // Export share sheet is hosted from inside the Batch sheet so we
        // never stack two sheets on the root NavigationStack at once.
        .sheet(isPresented: $isShowingExportShare) {
            ShareSheet(items: exportShareItems)
        }
        .fileImporter(
            isPresented: $isShowingProcessedImporter,
            allowedContentTypes: [.folder],
            allowsMultipleSelection: false
        ) { result in
            handleProcessedImport(result)
        }
    }

    private var batchSection: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack {
                Text("Batch Analysis")
                    .font(.headline)
                Spacer()
                if let track = session.analysisTrack {
                    Text("\(track.frameCount) frames")
                        .font(.caption.monospacedDigit())
                        .foregroundStyle(.secondary)
                }
            }

            if session.isProcessingAll {
                VStack(alignment: .leading, spacing: 6) {
                    ProgressView(value: session.processingProgress)
                    Text(String(format: "%.0f%% — keep the app foregrounded.", session.processingProgress * 100))
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
            } else {
                // Diagnostic A/B toggle: when ON the next "Process All Frames"
                // run uses bridge.solveID (no GRF estimation) instead of
                // bridge.solveIDGRF. Used to isolate whether observed muscle
                // asymmetry comes from the GRF estimator (asymmetric in
                // withGRF, symmetric in noGRF) or from elsewhere in the
                // pipeline. The mode is captured into the exported track so
                // re-loaded analyses know which path produced them.
                VStack(alignment: .leading, spacing: 4) {
                    Toggle("No-GRF mode (validation)", isOn: noGRFBinding)
                        .disabled(session.isProcessingAll)
                    Text("When enabled, ID runs without ground-reaction-force estimation. Useful for diagnosing GRF-related muscle asymmetry.")
                        .font(.caption2)
                        .foregroundStyle(.secondary)
                }

                HStack(spacing: 12) {
                    Button("Process All Frames") {
                        session.processAllFrames()
                    }
                    .buttonStyle(.borderedProminent)
                    .disabled(session.frameCount == 0)

                    if session.analysisTrack != nil {
                        Button("Export") {
                            do {
                                let output = try session.exportAnalysis()
                                exportShareItems = shareItems(for: output)
                                isShowingExportShare = true
                            } catch {
                                session.errorMessage = error.localizedDescription
                            }
                        }
                        .buttonStyle(.bordered)
                    }
                }

                // Skip the IK/ID/SO pipeline by re-loading a previously-
                // exported analysis folder. Useful after a fresh app
                // install (Documents is wiped) — keep the OfflineExport-…
                // folder in iCloud Drive / Files, then pick it here.
                VStack(alignment: .leading, spacing: 6) {
                    Button {
                        session.statusMessage = "Choose an OfflineExport-… folder containing *_processed.json"
                        isShowingProcessedImporter = true
                    } label: {
                        Label("Load Processed Analysis Folder…", systemImage: "tray.and.arrow.up")
                    }
                    .buttonStyle(.bordered)

                    Text("Pick the OfflineExport-… folder produced by Export to skip Process All Frames.")
                        .font(.caption2)
                        .foregroundStyle(.secondary)
                }

                if let track = session.analysisTrack {
                    HStack {
                        summaryBadge("Max |τ|/m", value: String(format: "%.2f Nm/kg", track.maxTorquePerKg))
                        summaryBadge("Root Res", value: String(format: "%.2f Nm/kg", track.maxRootResidualPerKg))
                        summaryBadge("Mass", value: String(format: "%.1f kg", track.totalMassKg))
                    }

                    let converged = track.samples.filter(\.muscleConverged).count
                    summaryBadge(
                        "SO Converged",
                        value: "\(converged) / \(track.frameCount) frames"
                    )
                }

                if let folder = session.lastExportFolder {
                    labeledValue("Export", value: folder.lastPathComponent)
                }
            }
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(16)
        .background(.thinMaterial, in: RoundedRectangle(cornerRadius: 16))
    }

    // MARK: - Helpers

    /// Bridges the enum `OfflineSession.idMode` to the `Bool` a SwiftUI
    /// `Toggle` expects. ON ⇒ `.noGRF`, OFF ⇒ `.withGRF`.
    private var noGRFBinding: Binding<Bool> {
        Binding(
            get: { session.idMode == .noGRF },
            set: { session.idMode = $0 ? .noGRF : .withGRF }
        )
    }

    private func autoOpenSourceIfFirstLaunch() {
        guard !hasSeenSourceSheet,
              session.modelURL == nil,
              session.videoURL == nil else { return }
        isShowingSourceSheet = true
        hasSeenSourceSheet = true
    }

    private func makeIDOutput(from sample: OfflineFrameSample) -> NimbleEngine.IDOutput {
        var out = NimbleEngine.IDOutput(
            jointTorques: sample.jointTorques,
            timestamp: sample.timestamp
        )
        out.leftFootForce = sample.leftFootForce
        out.rightFootForce = sample.rightFootForce
        out.leftFootInContact = sample.leftFootInContact
        out.rightFootInContact = sample.rightFootInContact
        out.rootResidualNorm = sample.rootResidualNorm
        return out
    }

    private func massNormalized(_ value: Double, mass: Double) -> String {
        guard mass > 0 else { return "—" }
        return String(format: "%.2f Nm/kg", value / mass)
    }

    private func footLabel(force: SIMD3<Double>, contact: Bool) -> String {
        guard contact else { return "swing" }
        let magnitude = sqrt(force.x * force.x + force.y * force.y + force.z * force.z)
        return String(format: "%.0f N", magnitude)
    }

    private var errorBinding: Binding<Bool> {
        Binding(
            get: { session.errorMessage != nil },
            set: { newValue in
                if !newValue {
                    session.errorMessage = nil
                }
            }
        )
    }

    /// Items handed to the system share sheet. We share the individual file
    /// URLs (not the folder) so AirDrop / Mail / Files all accept them — a
    /// folder URL would only work for "Save to Files". `processedJSONURL`
    /// is included so the user can save the round-trip blob to iCloud /
    /// Files and reload it after a reinstall via "Load Processed Analysis…".
    private func shareItems(for output: OfflineAnalysisExporter.Output) -> [URL] {
        var items: [URL] = [
            output.kinematicsMotURL,
            output.inverseDynamicsSTOURL,
            output.summaryJSONURL,
            output.processedJSONURL,
        ]
        if let activations = output.activationsSTOURL {
            items.append(activations)
        }
        if let forces = output.muscleForcesSTOURL {
            items.append(forces)
        }
        return items
    }

    private func handleFolderImport(_ result: Result<[URL], Error>) {
        isShowingFolderImporter = false
        switch result {
        case .success(let urls):
            guard let folder = urls.first else { return }
            session.importOpenCapFolder(from: folder)
        case .failure(let error):
            session.errorMessage = error.localizedDescription
        }
    }

    private func handleProcessedImport(_ result: Result<[URL], Error>) {
        isShowingProcessedImporter = false
        switch result {
        case .success(let urls):
            guard let folder = urls.first else { return }
            session.loadProcessedAnalysisFolder(from: folder)
        case .failure(let error):
            session.errorMessage = error.localizedDescription
        }
    }

    private func labeledValue(_ label: String, value: String) -> some View {
        HStack(alignment: .top) {
            Text(label)
                .font(.caption.weight(.semibold))
                .foregroundStyle(.secondary)
                .frame(width: 56, alignment: .leading)
            Text(value)
                .font(.caption)
                .textSelection(.enabled)
        }
    }

    /// Single-line label + single-line value at fixed font sizes — guarantees
    /// every badge collapses to the same intrinsic height regardless of
    /// content width, so the 2x3 grid stays visually rectangular instead of
    /// jagged when one column has a longer string than another.
    private func summaryBadge(_ label: String, value: String) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            Text(label)
                .font(.caption2)
                .foregroundStyle(.secondary)
                .lineLimit(1)
                .minimumScaleFactor(0.7)
            Text(value)
                .font(.callout.monospacedDigit())
                .foregroundStyle(.primary)
                .lineLimit(1)
                .minimumScaleFactor(0.6)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(10)
        .background(Color(.secondarySystemBackground), in: RoundedRectangle(cornerRadius: 12))
    }
}
