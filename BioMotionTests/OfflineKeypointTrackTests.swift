import XCTest
@testable import BioMotion

final class OfflineKeypointTrackTests: XCTestCase {

    private var fixtureFolder: URL!

    override func setUpWithError() throws {
        try super.setUpWithError()
        fixtureFolder = FileManager.default.temporaryDirectory
            .appendingPathComponent("OfflineKeypointTrackTests-\(UUID().uuidString)", isDirectory: true)
        try FileManager.default.createDirectory(at: fixtureFolder, withIntermediateDirectories: true)
    }

    override func tearDownWithError() throws {
        if let fixtureFolder, FileManager.default.fileExists(atPath: fixtureFolder.path) {
            try? FileManager.default.removeItem(at: fixtureFolder)
        }
        try super.tearDownWithError()
    }

    func testRoundTripsFrameMajorBinary() throws {
        let pointNames = ["A", "B", "C"]
        let frames: [[(Float, Float, Float)]] = [
            [(1, 2, 0.9), (3, 4, 0.8), (0, 0, 0)],   // frame 0; C is missing
            [(5, 6, 0.7), (7, 8, 0.6), (9, 10, 0.5)] // frame 1
        ]

        let urls = try writeFixture(
            videoFilename: "demo.mov",
            width: 1280,
            height: 720,
            fps: 60,
            pointNames: pointNames,
            frames: frames
        )

        let track = try OfflineKeypointTrack.load(metadataURL: urls.metadataURL)
        XCTAssertEqual(track.frameCount, frames.count)
        XCTAssertEqual(track.pointsPerFrame, pointNames.count)
        XCTAssertEqual(track.fps, 60)
        XCTAssertEqual(track.pixelWidth, 1280)
        XCTAssertEqual(track.pixelHeight, 720)
        XCTAssertEqual(track.pointNames, pointNames)

        let firstFrame = try XCTUnwrap(track.frame(at: 0))
        XCTAssertEqual(firstFrame[0].x, 1)
        XCTAssertEqual(firstFrame[0].y, 2)
        XCTAssertEqual(firstFrame[0].confidence, 0.9, accuracy: 1e-6)
        XCTAssertFalse(firstFrame[2].isPresent)

        let secondFrame = try XCTUnwrap(track.frame(at: 1))
        XCTAssertEqual(secondFrame[2].x, 9)
        XCTAssertEqual(secondFrame[2].y, 10)
        XCTAssertEqual(secondFrame[2].confidence, 0.5, accuracy: 1e-6)
    }

    func testFrameIndexQuantisesByFps() throws {
        let urls = try writeFixture(
            videoFilename: "demo.mov",
            width: 100,
            height: 100,
            fps: 30,
            pointNames: ["A"],
            frames: Array(repeating: [(0, 0, 1)], count: 10)
        )

        let track = try OfflineKeypointTrack.load(metadataURL: urls.metadataURL)
        XCTAssertEqual(track.frameIndex(for: 0.0), 0)
        XCTAssertEqual(track.frameIndex(for: 0.0166), 0)   // < half a frame at 30 Hz
        XCTAssertEqual(track.frameIndex(for: 0.034), 1)    // > half a frame
        XCTAssertEqual(track.frameIndex(for: 100.0), 9)    // clamps to last frame
        XCTAssertEqual(track.frameIndex(for: -1), 0)       // negative time clamps to zero
    }

    func testRejectsUnsupportedScalarType() throws {
        let urls = try writeFixture(
            videoFilename: "demo.mov",
            width: 10,
            height: 10,
            fps: 30,
            pointNames: ["A"],
            frames: [[(0, 0, 1)]],
            scalarType: "float64"
        )

        XCTAssertThrowsError(try OfflineKeypointTrack.load(metadataURL: urls.metadataURL)) { error in
            guard case OfflineKeypointTrack.LoadError.unsupportedScalarType = error else {
                return XCTFail("expected unsupportedScalarType, got \(error)")
            }
        }
    }

    func testAspectFitMappingLetterboxesAndScales() {
        let mapping = AspectFitMapping(pixelSize: CGSize(width: 1280, height: 720), in: CGSize(width: 800, height: 600))
        XCTAssertEqual(mapping.displayRect.width, 800, accuracy: 1e-6)
        XCTAssertEqual(mapping.displayRect.height, 450, accuracy: 1e-6)
        XCTAssertEqual(mapping.displayRect.minX, 0, accuracy: 1e-6)
        XCTAssertEqual(mapping.displayRect.minY, 75, accuracy: 1e-6)

        // Top-left pixel maps to the top-left of the display rect.
        let topLeft = mapping.point(forPixel: 0, 0)
        XCTAssertEqual(topLeft.x, 0, accuracy: 1e-6)
        XCTAssertEqual(topLeft.y, 75, accuracy: 1e-6)

        // Bottom-right pixel maps to the bottom-right of the display rect.
        let bottomRight = mapping.point(forPixel: 1280, 720)
        XCTAssertEqual(bottomRight.x, 800, accuracy: 1e-6)
        XCTAssertEqual(bottomRight.y, 525, accuracy: 1e-6)
    }

    // MARK: - Fixture helpers

    private func writeFixture(
        videoFilename: String,
        width: Int,
        height: Int,
        fps: Double,
        pointNames: [String],
        frames: [[(Float, Float, Float)]],
        scalarType: String = "float32"
    ) throws -> (metadataURL: URL, binaryURL: URL) {
        let metadataURL = fixtureFolder.appendingPathComponent("demo.keypoints.json")
        let binaryURL = fixtureFolder.appendingPathComponent("demo.keypoints.bin")

        var binary = Data()
        for frame in frames {
            XCTAssertEqual(frame.count, pointNames.count)
            for (x, y, c) in frame {
                var fx = x, fy = y, fc = c
                binary.append(Data(bytes: &fx, count: MemoryLayout<Float32>.size))
                binary.append(Data(bytes: &fy, count: MemoryLayout<Float32>.size))
                binary.append(Data(bytes: &fc, count: MemoryLayout<Float32>.size))
            }
        }
        try binary.write(to: binaryURL)

        let metadataDict: [String: Any] = [
            "format": OfflineKeypointTrack.supportedFormatTag,
            "video": [
                "filename": videoFilename,
                "width": width,
                "height": height,
                "fps": fps,
                "frame_count": frames.count
            ],
            "keypoint_set": [
                "name": "test-set",
                "points": pointNames
            ],
            "storage": [
                "binary_filename": binaryURL.lastPathComponent,
                "scalar_type": scalarType,
                "channels_per_point": 3,
                "channel_order": ["x", "y", "confidence"],
                "layout": "frame-major"
            ]
        ]

        let metadataData = try JSONSerialization.data(withJSONObject: metadataDict, options: [.prettyPrinted])
        try metadataData.write(to: metadataURL)

        return (metadataURL, binaryURL)
    }
}
