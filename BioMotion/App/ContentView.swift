import SwiftUI

struct ContentView: View {
    @StateObject private var bodyTracking = BodyTrackingSession()
    @StateObject private var recorder = MotionRecorder()
    @StateObject private var nimble = NimbleEngine()
    @State private var showExportSheet = false
    @State private var exportURLs: [URL] = []
    @State private var showIKPanel = false
    @State private var showCalibration = true
    @State private var showCharts = false
    @State private var showMuscleOverlay = true

    var body: some View {
        Group {
            if showCalibration {
                CalibrationView(
                    bodyTracking: bodyTracking,
                    nimble: nimble,
                    onComplete: {
                        withAnimation { showCalibration = false }
                    }
                )
            } else if showCharts {
                SessionChartsView(
                    nimble: nimble,
                    recorder: recorder,
                    onExport: { exportAll() },
                    onDismiss: { showCharts = false }
                )
            } else {
                trackingView
            }
        }
        .onAppear {
            bodyTracking.start()
            nimble.loadBundledModel()
        }
        .onChange(of: bodyTracking.currentFrame?.frameNumber) { _, _ in
            guard let frame = bodyTracking.currentFrame else { return }
            guard !showCalibration && !showCharts else { return }
            recorder.recordFrame(frame)
            nimble.processFrame(frame)
        }
        // Charts shown via export button, not auto-transition
        .sheet(isPresented: $showExportSheet) {
            ShareSheet(items: exportURLs)
        }
    }

    private var trackingView: some View {
        ZStack {
            // Camera feed
            SkeletonARView(
                session: bodyTracking.arSession,
                currentFrame: $bodyTracking.currentFrame,
                muscleActivations: nimble.lastMuscleResult?.activations,
                showMuscles: showMuscleOverlay
            )
            .ignoresSafeArea()

            // Single VStack overlay: top → spacer → bottom
            VStack(spacing: 0) {
                // === TOP BAR ===
                HStack {
                    StatusBadge(text: bodyTracking.trackingMessage, isActive: bodyTracking.isTracking)
                    if nimble.isModelLoaded {
                        StatusBadge(text: String(format: "IK %.1fms", nimble.ikSolveTimeMs), isActive: true)
                    }
                    Spacer()
                    if let frame = bodyTracking.currentFrame {
                        let tracked = frame.joints.filter(\.isTracked).count
                        StatusBadge(text: "\(tracked)/\(frame.joints.count)", isActive: tracked > 0)
                    }
                }
                .padding(.horizontal, 12)
                .padding(.top, 4)

                // Accuracy diagnostics: two rows of small pills.
                // Row 1 — IK residual, max joint torque per kg, model mass.
                // Row 2 — left/right foot load (fraction of body weight),
                //         GRF root residual. Target ranges after full overhaul:
                //   residual ≈ 0.01–0.03 m (ARKit-limited floor)
                //   |τ|/m    ≈ 1–3 Nm/kg for walking/squat (physiological)
                //   L+R load ≈ 1.0 ± 0.1 in stance (weight supported by feet)
                //   root res ≈ < 0.5 Nm/kg (GRF consistent with kinematics)
                if nimble.isModelLoaded && bodyTracking.isTracking {
                    VStack(alignment: .leading, spacing: 2) {
                        HStack(spacing: 6) {
                            AccuracyBadge(
                                label: "residual",
                                value: String(format: "%.3f m", nimble.ikMarkerResidualMeters),
                                good: nimble.ikMarkerResidualMeters < 0.05
                            )
                            AccuracyBadge(
                                label: "max |τ|/m",
                                value: String(format: "%.1f Nm/kg", nimble.maxTorquePerKg),
                                good: nimble.maxTorquePerKg < 5.0
                            )
                            if nimble.totalMassKg > 0 {
                                AccuracyBadge(
                                    label: "mass",
                                    value: String(format: "%.0f kg", nimble.totalMassKg),
                                    good: true
                                )
                            }
                            Spacer()
                        }
                        HStack(spacing: 6) {
                            let totalLoad = nimble.leftFootLoadFraction + nimble.rightFootLoadFraction
                            AccuracyBadge(
                                label: "L/R load",
                                value: String(format: "%.2f|%.2f", nimble.leftFootLoadFraction, nimble.rightFootLoadFraction),
                                good: abs(totalLoad - 1.0) < 0.3 || totalLoad < 0.1
                            )
                            AccuracyBadge(
                                label: "root res",
                                value: String(format: "%.2f Nm/kg", nimble.rootResidualPerKg),
                                good: nimble.rootResidualPerKg < 0.5
                            )
                            Spacer()
                        }
                    }
                    .padding(.horizontal, 12)
                    .padding(.top, 2)
                }

                // === PUSH EVERYTHING ELSE DOWN ===
                Spacer()

                // === BOTTOM SECTION ===

                // Data panels (only when tracking)
                if let muscle = nimble.lastMuscleResult {
                    MuscleActivationBar(muscle: muscle).padding(.horizontal, 12)
                }
                if showIKPanel, let ik = nimble.lastIKResult {
                    IKReadoutPanel(ikResult: ik, idResult: nimble.lastIDResult).padding(.horizontal, 12)
                } else if bodyTracking.isTracking, let frame = bodyTracking.currentFrame {
                    JointReadoutPanel(frame: frame).padding(.horizontal, 12)
                }

                // Toggle buttons
                if nimble.isModelLoaded && bodyTracking.isTracking {
                    HStack(spacing: 8) {
                        Button { withAnimation { showIKPanel.toggle() } } label: {
                            Text(showIKPanel ? "Positions" : "IK/ID")
                                .font(.caption2).foregroundStyle(.white)
                                .padding(.horizontal, 10).padding(.vertical, 4)
                                .background(.black.opacity(0.5), in: Capsule())
                        }
                        Button { showMuscleOverlay.toggle() } label: {
                            HStack(spacing: 4) {
                                Image(systemName: showMuscleOverlay ? "figure.run" : "figure.stand")
                                Text(showMuscleOverlay ? "Muscles ON" : "Muscles OFF")
                            }
                            .font(.caption2)
                            .foregroundStyle(showMuscleOverlay ? .green : .gray)
                            .padding(.horizontal, 10).padding(.vertical, 4)
                            .background(.black.opacity(0.5), in: Capsule())
                        }
                    }
                    .padding(.top, 4)
                }

                // === RECORD BAR (always at very bottom) ===
                HStack {
                    if recorder.isRecording {
                        HStack(spacing: 6) {
                            Circle().fill(.red).frame(width: 8, height: 8)
                            Text(formatDuration(recorder.duration)).monospacedDigit()
                            Text("\(recorder.recordedFrameCount)f")
                        }
                        .font(.caption).foregroundStyle(.white)
                        .padding(.horizontal, 10).padding(.vertical, 6)
                        .background(.black.opacity(0.6), in: Capsule())
                    } else if recorder.hasRecording {
                        Button { exportAll() } label: {
                            Label("Export", systemImage: "square.and.arrow.up")
                                .font(.caption).foregroundStyle(.white)
                                .padding(.horizontal, 10).padding(.vertical, 6)
                                .background(.blue.opacity(0.8), in: Capsule())
                        }
                    } else {
                        Color.clear.frame(width: 80, height: 1)
                    }

                    Spacer()

                    Button {
                        if recorder.isRecording {
                            recorder.stopRecording()
                            nimble.stopRecordingResults()
                        } else {
                            recorder.startRecording()
                            nimble.startRecordingResults()
                        }
                    } label: {
                        ZStack {
                            Circle().strokeBorder(.white, lineWidth: 3).frame(width: 64, height: 64)
                            if recorder.isRecording {
                                RoundedRectangle(cornerRadius: 4).fill(.red).frame(width: 24, height: 24)
                            } else {
                                Circle().fill(.red).frame(width: 52, height: 52)
                            }
                        }
                    }

                    Spacer()
                    Color.clear.frame(width: 80, height: 1)
                }
                .padding(.horizontal, 16)
                .padding(.top, 8)
                .padding(.bottom, 30)
            }
        }
    }

    private func exportAll() {
        var urls: [URL] = []
        var errors: [String] = []

        // Export .trc (marker positions)
        if recorder.hasRecording {
            let trcExporter = TRCExporter(frames: recorder.frames)
            do {
                let url = try trcExporter.export()
                urls.append(url)
            } catch {
                errors.append("TRC: \(error.localizedDescription)")
            }
        } else {
            errors.append("No recording data")
        }

        // Export .mot (joint angles from IK)
        do {
            let url = try nimble.exportMOT()
            urls.append(url)
        } catch {
            errors.append("MOT: no IK data")
        }

        // Export .sto (joint torques from ID)
        do {
            let url = try nimble.exportSTO()
            urls.append(url)
        } catch {
            errors.append("STO: no ID data")
        }

        if !urls.isEmpty {
            exportURLs = urls
            showExportSheet = true
        } else {
            // Nothing to export — show alert with reason
            // For now, export a summary text file so the share sheet isn't empty
            let summary = """
            BioMotion Export — No data available

            Reasons:
            \(errors.joined(separator: "\n"))

            Tips:
            - Make sure body tracking is active (green status)
            - Record for at least a few seconds
            - The Nimble model must load successfully
            """
            let url = FileManager.default.temporaryDirectory.appendingPathComponent("BioMotion_export_info.txt")
            try? summary.write(to: url, atomically: true, encoding: .utf8)
            exportURLs = [url]
            showExportSheet = true
        }
    }

    private func formatDuration(_ duration: TimeInterval) -> String {
        let minutes = Int(duration) / 60
        let seconds = Int(duration) % 60
        let fraction = Int((duration - Double(Int(duration))) * 10)
        return String(format: "%d:%02d.%d", minutes, seconds, fraction)
    }
}

// MARK: - Supporting Views

struct StatusBadge: View {
    let text: String
    let isActive: Bool

    var body: some View {
        HStack(spacing: 6) {
            Circle()
                .fill(isActive ? .green : .orange)
                .frame(width: 8, height: 8)
            Text(text)
                .font(.caption)
        }
        .foregroundStyle(.white)
        .padding(.horizontal, 10)
        .padding(.vertical, 5)
        .background(.black.opacity(0.6), in: Capsule())
    }
}

/// Compact label+value pill used for precision diagnostics in the HUD.
struct AccuracyBadge: View {
    let label: String
    let value: String
    let good: Bool

    var body: some View {
        HStack(spacing: 4) {
            Text(label).font(.system(size: 9, weight: .regular, design: .monospaced))
                .foregroundStyle(.white.opacity(0.7))
            Text(value).font(.system(size: 10, weight: .semibold, design: .monospaced))
                .foregroundStyle(good ? Color.green : Color.orange)
        }
        .padding(.horizontal, 8)
        .padding(.vertical, 3)
        .background(.black.opacity(0.55), in: Capsule())
    }
}

struct JointReadoutPanel: View {
    let frame: BodyFrame

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            Text("Joint Positions (m)")
                .font(.caption2.bold())
            ScrollView(.horizontal, showsIndicators: false) {
                HStack(spacing: 12) {
                    ForEach(frame.joints.filter(\.isTracked)) { joint in
                        VStack(alignment: .leading, spacing: 2) {
                            Text(joint.name)
                                .font(.caption2.bold())
                            Text(String(format: "%.2f, %.2f, %.2f",
                                        joint.worldPosition.x,
                                        joint.worldPosition.y,
                                        joint.worldPosition.z))
                                .font(.system(.caption2, design: .monospaced))
                        }
                    }
                }
            }
        }
        .foregroundStyle(.white)
        .padding(8)
        .background(.black.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }
}

struct IKReadoutPanel: View {
    let ikResult: NimbleEngine.IKOutput
    let idResult: NimbleEngine.IDOutput?

    // Show a curated set of important DOFs
    private let keyDOFs = [
        "hip_flexion_r", "hip_flexion_l",
        "knee_angle_r", "knee_angle_l",
        "ankle_angle_r", "ankle_angle_l",
        "lumbar_extension",
    ]

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            HStack {
                Text("IK Joint Angles")
                    .font(.caption2.bold())
                Spacer()
                Text(String(format: "err: %.1f mm", ikResult.error * 1000))
                    .font(.system(.caption2, design: .monospaced))
            }
            ScrollView(.horizontal, showsIndicators: false) {
                HStack(spacing: 12) {
                    ForEach(keyDOFs, id: \.self) { dof in
                        if let angle = ikResult.jointAngles[dof] {
                            VStack(alignment: .leading, spacing: 2) {
                                Text(shortName(dof))
                                    .font(.caption2.bold())
                                Text(String(format: "%.1f\u{00B0}", angle * 180.0 / .pi))
                                    .font(.system(.caption2, design: .monospaced))
                                if let torque = idResult?.jointTorques[dof] {
                                    Text(String(format: "%.1f Nm", torque))
                                        .font(.system(.caption2, design: .monospaced))
                                        .foregroundStyle(.yellow)
                                }
                            }
                        }
                    }
                }
            }
        }
        .foregroundStyle(.white)
        .padding(8)
        .background(.black.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }

    private func shortName(_ dof: String) -> String {
        dof.replacingOccurrences(of: "_r", with: " R")
           .replacingOccurrences(of: "_l", with: " L")
           .replacingOccurrences(of: "_", with: " ")
           .capitalized
    }
}

struct MuscleActivationBar: View {
    let muscle: NimbleEngine.MuscleOutput

    // Show top activated muscles
    private let keyMuscles = [
        "soleus_r", "soleus_l", "gasmed_r", "gasmed_l",
        "tibant_r", "tibant_l", "vasmed_r", "vasmed_l",
        "recfem_r", "recfem_l", "glmax1_r", "glmax1_l",
    ]

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            Text("Muscle Activations")
                .font(.caption2.bold())
            ScrollView(.horizontal, showsIndicators: false) {
                HStack(spacing: 6) {
                    ForEach(keyMuscles, id: \.self) { name in
                        if let activation = muscle.activations[name] {
                            VStack(spacing: 2) {
                                // Activation bar
                                RoundedRectangle(cornerRadius: 2)
                                    .fill(activationColor(activation))
                                    .frame(width: 20, height: CGFloat(activation * 40))
                                    .frame(height: 40, alignment: .bottom)
                                Text(shortMuscleName(name))
                                    .font(.system(size: 7))
                                Text(String(format: "%.0f%%", activation * 100))
                                    .font(.system(size: 7, design: .monospaced))
                            }
                        }
                    }
                }
            }
        }
        .foregroundStyle(.white)
        .padding(8)
        .background(.black.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }

    private func activationColor(_ a: Double) -> Color {
        if a < 0.3 { return .blue }
        if a < 0.6 { return .green }
        if a < 0.8 { return .yellow }
        return .red
    }

    private func shortMuscleName(_ name: String) -> String {
        name.replacingOccurrences(of: "_r", with: "R")
            .replacingOccurrences(of: "_l", with: "L")
    }
}

/// UIKit share sheet wrapper.
struct ShareSheet: UIViewControllerRepresentable {
    let items: [Any]

    func makeUIViewController(context: Context) -> UIActivityViewController {
        UIActivityViewController(activityItems: items, applicationActivities: nil)
    }

    func updateUIViewController(_ uiViewController: UIActivityViewController, context: Context) {}
}
