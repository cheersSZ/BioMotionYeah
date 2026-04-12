import XCTest
@testable import BioMotion

final class TRCExporterTests: XCTestCase {

    private func makeJoints(tracked: Bool = true) -> [TrackedJoint] {
        JointMapping.primary.map { mapping in
            TrackedJoint(
                id: mapping.arkitName,
                name: mapping.displayName,
                worldPosition: SIMD3<Float>(0.1, 0.9, 0.0),
                isTracked: tracked
            )
        }
    }

    private func makeFrames(count: Int, fps: Double = 60.0) -> [BodyFrame] {
        (0..<count).map { i in
            BodyFrame(
                timestamp: Double(i) / fps,
                frameNumber: i + 1,
                joints: makeJoints()
            )
        }
    }

    // MARK: - Header

    func testTRCHeaderLine1() {
        let exporter = TRCExporter(frames: makeFrames(count: 3))
        let output = exporter.generate()
        let lines = output.components(separatedBy: "\n")

        XCTAssertTrue(lines[0].hasPrefix("PathFileType\t4\t(X/Y/Z)"))
    }

    func testTRCHeaderLine2MetadataKeys() {
        let exporter = TRCExporter(frames: makeFrames(count: 3))
        let output = exporter.generate()
        let lines = output.components(separatedBy: "\n")

        let expectedKeys = "DataRate\tCameraRate\tNumFrames\tNumMarkers\tUnits\tOrigDataRate\tOrigDataStartFrame\tOrigNumFrames"
        XCTAssertEqual(lines[1], expectedKeys)
    }

    func testTRCHeaderLine3Values() {
        let exporter = TRCExporter(frames: makeFrames(count: 5))
        let output = exporter.generate()
        let lines = output.components(separatedBy: "\n")
        let values = lines[2].components(separatedBy: "\t")

        // NumFrames
        XCTAssertEqual(values[2], "5")
        // NumMarkers
        XCTAssertEqual(values[3], "\(JointMapping.primary.count)")
        // Units
        XCTAssertEqual(values[4], "m")
    }

    func testTRCHeaderLine4MarkerNames() {
        let exporter = TRCExporter(frames: makeFrames(count: 1))
        let output = exporter.generate()
        let lines = output.components(separatedBy: "\n")

        XCTAssertTrue(lines[3].hasPrefix("Frame#\tTime"))
        // Should contain all OpenSim marker names
        for mapping in JointMapping.primary {
            XCTAssertTrue(lines[3].contains(mapping.opensimName),
                          "Header should contain marker \(mapping.opensimName)")
        }
    }

    func testTRCHeaderLine5ComponentLabels() {
        let exporter = TRCExporter(frames: makeFrames(count: 1))
        let output = exporter.generate()
        let lines = output.components(separatedBy: "\n")

        // Line 5 should have X/Y/Z component labels
        XCTAssertTrue(lines[4].contains("X1"))
        XCTAssertTrue(lines[4].contains("Y1"))
        XCTAssertTrue(lines[4].contains("Z1"))
    }

    func testTRCHeaderLine6Blank() {
        let exporter = TRCExporter(frames: makeFrames(count: 1))
        let output = exporter.generate()
        let lines = output.components(separatedBy: "\n")

        XCTAssertEqual(lines[5], "", "Line 6 should be blank")
    }

    // MARK: - Data rows

    func testTRCDataRowCount() {
        let exporter = TRCExporter(frames: makeFrames(count: 10))
        let output = exporter.generate()
        let lines = output.components(separatedBy: "\n")

        // 6 header lines + 10 data rows
        XCTAssertEqual(lines.count, 16)
    }

    func testTRCDataRowFormat() {
        let joints = JointMapping.primary.enumerated().map { (i, mapping) in
            TrackedJoint(
                id: mapping.arkitName,
                name: mapping.displayName,
                worldPosition: SIMD3<Float>(Float(i) * 0.1, 1.0, 0.0),
                isTracked: true
            )
        }
        let frames = [BodyFrame(timestamp: 0.0, frameNumber: 1, joints: joints)]
        let exporter = TRCExporter(frames: frames)
        let output = exporter.generate()
        let lines = output.components(separatedBy: "\n")
        let dataRow = lines[6] // First data row
        let columns = dataRow.components(separatedBy: "\t")

        // Frame number
        XCTAssertEqual(columns[0], "1")
        // Time
        XCTAssertEqual(columns[1], "0.000000")
        // First marker X (index 0 * 0.1 = 0.0)
        XCTAssertEqual(columns[2], "0.000000")
        // First marker Y
        XCTAssertEqual(columns[3], "1.000000")
    }

    func testTRCTimeStartsAtZero() {
        let frames = [
            BodyFrame(timestamp: 100.5, frameNumber: 1, joints: makeJoints()),
            BodyFrame(timestamp: 100.55, frameNumber: 2, joints: makeJoints()),
        ]
        let exporter = TRCExporter(frames: frames)
        let output = exporter.generate()
        let lines = output.components(separatedBy: "\n")

        let row1Cols = lines[6].components(separatedBy: "\t")
        let row2Cols = lines[7].components(separatedBy: "\t")

        // Time should be relative to first frame
        XCTAssertEqual(row1Cols[1], "0.000000")
        XCTAssertTrue(row2Cols[1].hasPrefix("0.05"))
    }

    func testTRCUntrackedMarkersEmpty() {
        let joints = JointMapping.primary.map { mapping in
            TrackedJoint(
                id: mapping.arkitName,
                name: mapping.displayName,
                worldPosition: .zero,
                isTracked: false
            )
        }
        let frames = [BodyFrame(timestamp: 0.0, frameNumber: 1, joints: joints)]
        let exporter = TRCExporter(frames: frames)
        let output = exporter.generate()
        let lines = output.components(separatedBy: "\n")
        let dataRow = lines[6]

        // After Frame# and Time, untracked markers should produce empty tab-separated fields
        let columns = dataRow.components(separatedBy: "\t")
        // Columns 2,3,4 should be empty (first untracked marker)
        XCTAssertEqual(columns[2], "")
        XCTAssertEqual(columns[3], "")
        XCTAssertEqual(columns[4], "")
    }

    // MARK: - Export

    func testTRCExportCreatesFile() throws {
        let exporter = TRCExporter(frames: makeFrames(count: 3))
        let url = try exporter.export(filename: "test_export")

        XCTAssertTrue(FileManager.default.fileExists(atPath: url.path))
        XCTAssertTrue(url.lastPathComponent.hasSuffix(".trc"))

        // Clean up
        try? FileManager.default.removeItem(at: url)
    }

    func testTRCExportedFileIsReadable() throws {
        let exporter = TRCExporter(frames: makeFrames(count: 2))
        let url = try exporter.export(filename: "test_readable")
        let content = try String(contentsOf: url, encoding: .utf8)

        XCTAssertTrue(content.hasPrefix("PathFileType"))
        XCTAssertTrue(content.contains("DataRate"))

        try? FileManager.default.removeItem(at: url)
    }

    // MARK: - Edge cases

    func testTRCEmptyFrames() {
        let exporter = TRCExporter(frames: [])
        let output = exporter.generate()
        XCTAssertEqual(output, "")
    }

    func testTRCSingleFrame() {
        let exporter = TRCExporter(frames: makeFrames(count: 1))
        let output = exporter.generate()
        let lines = output.components(separatedBy: "\n")

        // 6 header lines + 1 data row
        XCTAssertEqual(lines.count, 7)
    }
}
