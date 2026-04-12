import XCTest
@testable import BioMotion

final class MotionRecorderTests: XCTestCase {

    private func makeFrame(timestamp: TimeInterval, frameNumber: Int) -> BodyFrame {
        let joints = [
            TrackedJoint(id: "hips_joint", name: "Pelvis", worldPosition: SIMD3(0, 1, 0), isTracked: true)
        ]
        return BodyFrame(timestamp: timestamp, frameNumber: frameNumber, joints: joints)
    }

    func testInitialState() {
        let recorder = MotionRecorder()
        XCTAssertFalse(recorder.isRecording)
        XCTAssertFalse(recorder.hasRecording)
        XCTAssertEqual(recorder.recordedFrameCount, 0)
        XCTAssertEqual(recorder.duration, 0)
    }

    func testStartRecording() {
        let recorder = MotionRecorder()
        recorder.startRecording()
        XCTAssertTrue(recorder.isRecording)
        XCTAssertEqual(recorder.recordedFrameCount, 0)
    }

    func testRecordFrames() {
        let recorder = MotionRecorder()
        recorder.startRecording()

        recorder.recordFrame(makeFrame(timestamp: 0.0, frameNumber: 1))
        recorder.recordFrame(makeFrame(timestamp: 0.016, frameNumber: 2))
        recorder.recordFrame(makeFrame(timestamp: 0.033, frameNumber: 3))

        XCTAssertEqual(recorder.recordedFrameCount, 3)
        XCTAssertEqual(recorder.frames.count, 3)
        XCTAssertEqual(recorder.duration, 0.033, accuracy: 0.001)
    }

    func testIgnoresFramesWhenNotRecording() {
        let recorder = MotionRecorder()

        recorder.recordFrame(makeFrame(timestamp: 0.0, frameNumber: 1))
        recorder.recordFrame(makeFrame(timestamp: 0.016, frameNumber: 2))

        XCTAssertEqual(recorder.recordedFrameCount, 0)
        XCTAssertFalse(recorder.hasRecording)
    }

    func testStopRecording() {
        let recorder = MotionRecorder()
        recorder.startRecording()
        recorder.recordFrame(makeFrame(timestamp: 0.0, frameNumber: 1))
        recorder.recordFrame(makeFrame(timestamp: 0.016, frameNumber: 2))
        recorder.stopRecording()

        XCTAssertFalse(recorder.isRecording)
        XCTAssertTrue(recorder.hasRecording)
        XCTAssertEqual(recorder.recordedFrameCount, 2)
    }

    func testStopThenRecordIgnoresFrames() {
        let recorder = MotionRecorder()
        recorder.startRecording()
        recorder.recordFrame(makeFrame(timestamp: 0.0, frameNumber: 1))
        recorder.stopRecording()

        recorder.recordFrame(makeFrame(timestamp: 0.016, frameNumber: 2))
        XCTAssertEqual(recorder.recordedFrameCount, 1)
    }

    func testStartClearsOldRecording() {
        let recorder = MotionRecorder()
        recorder.startRecording()
        recorder.recordFrame(makeFrame(timestamp: 0.0, frameNumber: 1))
        recorder.recordFrame(makeFrame(timestamp: 0.016, frameNumber: 2))
        recorder.stopRecording()

        recorder.startRecording()
        XCTAssertEqual(recorder.recordedFrameCount, 0)
        XCTAssertFalse(recorder.hasRecording)
    }

    func testAverageFPS() {
        let recorder = MotionRecorder()
        recorder.startRecording()

        // 61 frames over 1 second = 60 fps
        for i in 0...60 {
            recorder.recordFrame(makeFrame(timestamp: Double(i) / 60.0, frameNumber: i + 1))
        }

        XCTAssertEqual(recorder.averageFPS, 60.0, accuracy: 0.1)
    }

    func testAverageFPSSingleFrame() {
        let recorder = MotionRecorder()
        recorder.startRecording()
        recorder.recordFrame(makeFrame(timestamp: 0.0, frameNumber: 1))
        XCTAssertEqual(recorder.averageFPS, 0)
    }

    func testDurationWithOffset() {
        let recorder = MotionRecorder()
        recorder.startRecording()

        // Timestamps don't start at zero (simulating real ARKit timestamps)
        recorder.recordFrame(makeFrame(timestamp: 100.0, frameNumber: 1))
        recorder.recordFrame(makeFrame(timestamp: 100.5, frameNumber: 2))
        recorder.recordFrame(makeFrame(timestamp: 101.0, frameNumber: 3))

        XCTAssertEqual(recorder.duration, 1.0, accuracy: 0.001)
    }
}
