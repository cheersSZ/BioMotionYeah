import Foundation

/// Records body frames over a session for later export.
final class MotionRecorder: ObservableObject {
    @Published var isRecording = false
    @Published var recordedFrameCount = 0
    @Published var duration: TimeInterval = 0

    private(set) var frames: [BodyFrame] = []
    private var startTime: TimeInterval?

    func startRecording() {
        frames.removeAll()
        recordedFrameCount = 0
        duration = 0
        startTime = nil
        isRecording = true
    }

    func stopRecording() {
        isRecording = false
    }

    func recordFrame(_ frame: BodyFrame) {
        guard isRecording else { return }

        if startTime == nil {
            startTime = frame.timestamp
        }

        frames.append(frame)
        recordedFrameCount = frames.count
        duration = frame.timestamp - (startTime ?? frame.timestamp)
    }

    var hasRecording: Bool {
        !frames.isEmpty
    }

    /// Average frame rate of the recording.
    var averageFPS: Double {
        guard frames.count > 1,
              let first = frames.first,
              let last = frames.last else { return 0 }
        let elapsed = last.timestamp - first.timestamp
        guard elapsed > 0 else { return 0 }
        return Double(frames.count - 1) / elapsed
    }
}
