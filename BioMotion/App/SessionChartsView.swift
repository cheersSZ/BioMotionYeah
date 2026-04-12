import SwiftUI
import Charts

/// Post-session charts showing joint angles and torques over time.
struct SessionChartsView: View {
    let nimble: NimbleEngine
    let recorder: MotionRecorder
    let onExport: () -> Void
    let onDismiss: () -> Void

    @State private var selectedDOF = "hip_flexion_r"
    @State private var showTorques = false

    private let keyDOFs = [
        "hip_flexion_r", "hip_flexion_l",
        "knee_angle_r", "knee_angle_l",
        "ankle_angle_r", "ankle_angle_l",
        "lumbar_extension",
        "hip_adduction_r", "hip_adduction_l",
        "hip_rotation_r", "hip_rotation_l",
    ]

    var body: some View {
        NavigationView {
            ScrollView {
                VStack(alignment: .leading, spacing: 16) {
                    // Session summary
                    sessionSummary
                        .padding(.horizontal)

                    // DOF picker
                    ScrollView(.horizontal, showsIndicators: false) {
                        HStack(spacing: 8) {
                            ForEach(availableDOFs, id: \.self) { dof in
                                Button {
                                    selectedDOF = dof
                                } label: {
                                    Text(shortName(dof))
                                        .font(.caption)
                                        .padding(.horizontal, 10)
                                        .padding(.vertical, 6)
                                        .background(
                                            selectedDOF == dof ? Color.blue : Color.gray.opacity(0.3),
                                            in: Capsule()
                                        )
                                        .foregroundStyle(selectedDOF == dof ? .white : .primary)
                                }
                            }
                        }
                        .padding(.horizontal)
                    }

                    // Joint angle chart
                    if !nimble.ikHistory.isEmpty {
                        chartSection(title: "Joint Angle (\u{00B0})", data: angleData)
                            .padding(.horizontal)
                    }

                    // Toggle torques
                    if !nimble.idHistory.isEmpty {
                        Toggle("Show Joint Torques", isOn: $showTorques)
                            .padding(.horizontal)

                        if showTorques {
                            chartSection(title: "Joint Torque (Nm)", data: torqueData)
                                .padding(.horizontal)
                        }
                    }

                    // IK error chart
                    if !nimble.ikHistory.isEmpty {
                        ikErrorChart
                            .padding(.horizontal)
                    }
                }
                .padding(.vertical)
            }
            .navigationTitle("Session Analysis")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .topBarLeading) {
                    Button("Close") { onDismiss() }
                }
                ToolbarItem(placement: .topBarTrailing) {
                    Button {
                        onExport()
                    } label: {
                        Label("Export", systemImage: "square.and.arrow.up")
                    }
                }
            }
        }
    }

    // MARK: - Session Summary

    private var sessionSummary: some View {
        HStack(spacing: 16) {
            SummaryStat(label: "Duration", value: String(format: "%.1fs", recorder.duration))
            SummaryStat(label: "Frames", value: "\(recorder.recordedFrameCount)")
            SummaryStat(label: "FPS", value: String(format: "%.0f", recorder.averageFPS))
            if !nimble.ikHistory.isEmpty {
                let avgError = nimble.ikHistory.map(\.error).reduce(0, +) / Double(nimble.ikHistory.count)
                SummaryStat(label: "Avg IK Err", value: String(format: "%.1f mm", avgError * 1000))
            }
        }
        .padding()
        .background(Color(.systemGray6), in: RoundedRectangle(cornerRadius: 12))
    }

    // MARK: - Chart Data

    private var availableDOFs: [String] {
        guard let first = nimble.ikHistory.first else { return keyDOFs }
        return keyDOFs.filter { first.angles.keys.contains($0) }
    }

    private struct ChartPoint: Identifiable {
        let id = UUID()
        let time: Double
        let value: Double
    }

    private var angleData: [ChartPoint] {
        guard !nimble.ikHistory.isEmpty else { return [] }
        let startTime = nimble.ikHistory.first!.timestamp
        return nimble.ikHistory.compactMap { entry in
            guard let angle = entry.angles[selectedDOF] else { return nil }
            return ChartPoint(
                time: entry.timestamp - startTime,
                value: angle * 180.0 / .pi
            )
        }
    }

    private var torqueData: [ChartPoint] {
        guard !nimble.idHistory.isEmpty else { return [] }
        let startTime = nimble.idHistory.first!.timestamp
        return nimble.idHistory.compactMap { entry in
            guard let torque = entry.jointTorques[selectedDOF] else { return nil }
            return ChartPoint(
                time: entry.timestamp - startTime,
                value: torque
            )
        }
    }

    // MARK: - Chart Views

    private func chartSection(title: String, data: [ChartPoint]) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            Text("\(shortName(selectedDOF)) — \(title)")
                .font(.headline)

            if data.isEmpty {
                Text("No data for this DOF")
                    .font(.caption)
                    .foregroundStyle(.secondary)
                    .frame(height: 200)
            } else {
                Chart(data) { point in
                    LineMark(
                        x: .value("Time (s)", point.time),
                        y: .value(title, point.value)
                    )
                    .foregroundStyle(.blue)
                    .lineStyle(StrokeStyle(lineWidth: 1.5))
                }
                .chartXAxisLabel("Time (s)")
                .chartYAxisLabel(title)
                .frame(height: 200)
                .padding(4)
                .background(Color(.systemGray6), in: RoundedRectangle(cornerRadius: 8))
            }
        }
    }

    private var ikErrorChart: some View {
        VStack(alignment: .leading, spacing: 8) {
            Text("IK Marker Error")
                .font(.headline)

            let startTime = nimble.ikHistory.first!.timestamp
            let errorData = nimble.ikHistory.map { entry in
                ChartPoint(
                    time: entry.timestamp - startTime,
                    value: entry.error * 1000  // Convert to mm
                )
            }

            Chart(errorData) { point in
                LineMark(
                    x: .value("Time (s)", point.time),
                    y: .value("Error (mm)", point.value)
                )
                .foregroundStyle(.orange)
                .lineStyle(StrokeStyle(lineWidth: 1.5))
            }
            .chartXAxisLabel("Time (s)")
            .chartYAxisLabel("Error (mm)")
            .frame(height: 150)
            .padding(4)
            .background(Color(.systemGray6), in: RoundedRectangle(cornerRadius: 8))
        }
    }

    // MARK: - Helpers

    private func shortName(_ dof: String) -> String {
        dof.replacingOccurrences(of: "_r", with: " R")
           .replacingOccurrences(of: "_l", with: " L")
           .replacingOccurrences(of: "_", with: " ")
           .capitalized
    }
}

struct SummaryStat: View {
    let label: String
    let value: String

    var body: some View {
        VStack(spacing: 4) {
            Text(value)
                .font(.headline.monospacedDigit())
            Text(label)
                .font(.caption2)
                .foregroundStyle(.secondary)
        }
        .frame(maxWidth: .infinity)
    }
}
