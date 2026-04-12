import SwiftUI
import ARKit
import RealityKit

/// T-pose calibration flow with live camera preview.
/// User sees themselves, positions into T-pose, then taps Capture.
struct CalibrationView: View {
    @ObservedObject var bodyTracking: BodyTrackingSession
    @ObservedObject var nimble: NimbleEngine
    let onComplete: () -> Void

    @State private var phase: CalibrationPhase = .livePreview
    @State private var capturedHeight: Double?
    @State private var calibrationFrames: [BodyFrame] = []
    @State private var isCapturing = false
    @State private var timer: Timer?

    enum CalibrationPhase {
        case livePreview   // Live camera + skeleton, user positions themselves
        case capturing     // Brief capture (~2s)
        case done          // Show results
    }

    var body: some View {
        ZStack {
            // Live camera feed with skeleton — always visible
            SkeletonARView(
                session: bodyTracking.arSession,
                currentFrame: $bodyTracking.currentFrame
            )
            .ignoresSafeArea()

            // Overlay UI on top of camera
            VStack(spacing: 0) {
                // Top: status + instructions
                VStack(spacing: 8) {
                    Text("Calibration")
                        .font(.title2.bold())
                        .foregroundStyle(.white)

                    if bodyTracking.isTracking {
                        HStack(spacing: 6) {
                            Circle().fill(.green).frame(width: 8, height: 8)
                            Text("Body detected")
                                .font(.caption)
                                .foregroundStyle(.green)
                        }
                    } else {
                        HStack(spacing: 6) {
                            Circle().fill(.orange).frame(width: 8, height: 8)
                            Text("Looking for body...")
                                .font(.caption)
                                .foregroundStyle(.orange)
                        }
                    }

                    Text("Stand in a T-pose facing the camera")
                        .font(.callout)
                        .foregroundStyle(.white.opacity(0.8))
                }
                .padding(.top, 16)
                .padding(.horizontal)
                .background(
                    LinearGradient(colors: [.black.opacity(0.7), .clear],
                                   startPoint: .top, endPoint: .bottom)
                    .ignoresSafeArea()
                )

                Spacer()

                // Center: T-pose guide overlay (when not yet tracking)
                if !bodyTracking.isTracking {
                    Image(systemName: "figure.arms.open")
                        .font(.system(size: 120))
                        .foregroundStyle(.white.opacity(0.2))
                }

                Spacer()

                // Bottom: action buttons
                VStack(spacing: 12) {
                    switch phase {
                    case .livePreview:
                        // Capture button — only enabled when body is tracked
                        Button {
                            startCapturing()
                        } label: {
                            HStack(spacing: 8) {
                                Image(systemName: "camera.fill")
                                Text("Capture T-Pose")
                            }
                            .font(.headline)
                            .foregroundStyle(.white)
                            .padding(.horizontal, 32)
                            .padding(.vertical, 14)
                            .background(
                                bodyTracking.isTracking ? Color.blue : Color.gray,
                                in: RoundedRectangle(cornerRadius: 12)
                            )
                        }
                        .disabled(!bodyTracking.isTracking)

                        Button("Skip Calibration") {
                            onComplete()
                        }
                        .foregroundStyle(.white.opacity(0.5))
                        .font(.callout)

                    case .capturing:
                        HStack(spacing: 12) {
                            ProgressView()
                                .tint(.white)
                            Text("Capturing... \(calibrationFrames.count)/60 frames")
                                .font(.callout)
                                .foregroundStyle(.white)
                        }
                        .padding(.horizontal, 24)
                        .padding(.vertical, 12)
                        .background(.blue.opacity(0.8), in: Capsule())

                        Text("Hold your pose")
                            .font(.caption)
                            .foregroundStyle(.white.opacity(0.6))

                    case .done:
                        VStack(spacing: 8) {
                            Image(systemName: "checkmark.circle.fill")
                                .font(.system(size: 40))
                                .foregroundStyle(.green)

                            if let height = capturedHeight {
                                Text(String(format: "Height: %.2f m", height))
                                    .font(.headline)
                                    .foregroundStyle(.white)
                            }

                            Text("Model scaled to your body")
                                .font(.caption)
                                .foregroundStyle(.white.opacity(0.7))
                        }

                        HStack(spacing: 16) {
                            Button {
                                phase = .livePreview
                                calibrationFrames.removeAll()
                            } label: {
                                Text("Redo")
                                    .font(.callout)
                                    .foregroundStyle(.white)
                                    .padding(.horizontal, 24)
                                    .padding(.vertical, 10)
                                    .background(.gray.opacity(0.6), in: RoundedRectangle(cornerRadius: 10))
                            }

                            Button {
                                onComplete()
                            } label: {
                                HStack(spacing: 6) {
                                    Text("Start Tracking")
                                    Image(systemName: "arrow.right")
                                }
                                .font(.headline)
                                .foregroundStyle(.white)
                                .padding(.horizontal, 24)
                                .padding(.vertical, 10)
                                .background(.green, in: RoundedRectangle(cornerRadius: 10))
                            }
                        }
                    }
                }
                .padding(.bottom, 40)
                .padding(.horizontal)
                .background(
                    LinearGradient(colors: [.clear, .black.opacity(0.7)],
                                   startPoint: .top, endPoint: .bottom)
                    .ignoresSafeArea()
                )
            }
        }
    }

    // MARK: - Calibration Logic

    private func startCapturing() {
        phase = .capturing
        calibrationFrames.removeAll()

        timer = Timer.scheduledTimer(withTimeInterval: 1.0 / 30.0, repeats: true) { _ in
            if let frame = bodyTracking.currentFrame {
                calibrationFrames.append(frame)
            }

            if calibrationFrames.count >= 60 {
                timer?.invalidate()
                processCalibration()
            }
        }
    }

    private func processCalibration() {
        guard !calibrationFrames.isEmpty else {
            phase = .livePreview
            return
        }

        let trackedFrames = calibrationFrames.filter { frame in
            frame.joints.contains(where: { $0.id == "hips_joint" && $0.isTracked })
        }

        guard !trackedFrames.isEmpty else {
            phase = .livePreview
            return
        }

        let height = estimateHeight(from: trackedFrames)
        capturedHeight = height

        var avgPositions: [Float] = []
        var markerNames: [String] = []

        for mapping in JointMapping.primary {
            var sumX: Float = 0, sumY: Float = 0, sumZ: Float = 0
            var count = 0

            for frame in trackedFrames {
                if let joint = frame.joints.first(where: { $0.id == mapping.arkitName }),
                   joint.isTracked {
                    sumX += joint.worldPosition.x
                    sumY += joint.worldPosition.y
                    sumZ += joint.worldPosition.z
                    count += 1
                }
            }

            if count > 0 {
                markerNames.append(mapping.opensimName)
                avgPositions.append(sumX / Float(count))
                avgPositions.append(sumY / Float(count))
                avgPositions.append(sumZ / Float(count))
            }
        }

        nimble.scaleModel(height: height, markerPositions: avgPositions, markerNames: markerNames)
        phase = .done
    }

    private func estimateHeight(from frames: [BodyFrame]) -> Double {
        var heights: [Float] = []

        for frame in frames {
            let head = frame.joints.first(where: { $0.id == "head_joint" })
            let leftAnkle = frame.joints.first(where: { $0.id == "left_foot_joint" })
            let rightAnkle = frame.joints.first(where: { $0.id == "right_foot_joint" })

            guard let h = head, h.isTracked else { continue }

            let ankleY: Float
            if let la = leftAnkle, la.isTracked, let ra = rightAnkle, ra.isTracked {
                ankleY = min(la.worldPosition.y, ra.worldPosition.y)
            } else if let la = leftAnkle, la.isTracked {
                ankleY = la.worldPosition.y
            } else if let ra = rightAnkle, ra.isTracked {
                ankleY = ra.worldPosition.y
            } else {
                continue
            }

            let height = h.worldPosition.y - ankleY + 0.08
            if height > 1.0 && height < 2.5 {
                heights.append(height)
            }
        }

        guard !heights.isEmpty else { return 1.75 }
        return Double(heights.reduce(0, +) / Float(heights.count))
    }
}
