import XCTest
@testable import BioMotion

final class ImportedMotionTests: XCTestCase {

    func testMOTParserReadsColumnsAndFrames() throws {
        let text = """
        Coordinates
        version=1
        nRows=2
        nColumns=4
        inDegrees=yes
        endheader
        time\tpelvis_tilt\tpelvis_tx\thip_flexion_r
        0.0000\t10.0\t1.2\t20.0
        0.0100\t11.0\t1.3\t21.0
        """

        let series = try MOTParser.parse(text: text)
        XCTAssertEqual(series.coordinateNames, ["pelvis_tilt", "pelvis_tx", "hip_flexion_r"])
        XCTAssertTrue(series.rotationalValuesAreDegrees)
        XCTAssertEqual(series.frames.count, 2)
        XCTAssertEqual(series.frames[0].timestamp, 0.0, accuracy: 1e-9)
        let pelvisTx = try XCTUnwrap(series.frames[1].coordinates["pelvis_tx"])
        XCTAssertEqual(pelvisTx, 1.3, accuracy: 1e-9)
    }

    func testPreparationConvertsRotationsToRadiansButKeepsTranslations() throws {
        let text = """
        Coordinates
        version=1
        nRows=1
        nColumns=4
        inDegrees=yes
        endheader
        time\tpelvis_tilt\tpelvis_tx\thip_flexion_r
        0.0000\t180.0\t1.25\t90.0
        """

        let series = try MOTParser.parse(text: text)
        let prepared = ImportedMotionPreparer.prepare(
            series: series,
            modelDOFNames: ["pelvis_tilt", "pelvis_tx", "hip_flexion_r"]
        )

        XCTAssertEqual(prepared.frames.count, 1)
        let frame = try XCTUnwrap(prepared.frames.first)
        XCTAssertEqual(frame.positions[0], .pi, accuracy: 1e-9)
        XCTAssertEqual(frame.positions[1], 1.25, accuracy: 1e-9)
        XCTAssertEqual(frame.positions[2], .pi / 2.0, accuracy: 1e-9)
    }

    func testPreparationUsesKnownCoordinateAliases() throws {
        let text = """
        Coordinates
        version=1
        nRows=1
        nColumns=2
        inDegrees=yes
        endheader
        time\tlumbar_extension
        0.0000\t90.0
        """

        let series = try MOTParser.parse(text: text)
        let prepared = ImportedMotionPreparer.prepare(
            series: series,
            modelDOFNames: ["Abs_FE"]
        )

        XCTAssertTrue(prepared.unmatchedSourceCoordinates.isEmpty)
        XCTAssertTrue(prepared.unmatchedModelDOFs.isEmpty)
        XCTAssertEqual(prepared.frames[0].positions[0], .pi / 2.0, accuracy: 1e-9)
    }

    func testTrail2MotionCoordinatesMatchOpenCapModel() throws {
        let datasetURL = try trail2DatasetURL()
        let motionURL = datasetURL.appendingPathComponent("run2.mot")
        let modelURL = datasetURL.appendingPathComponent("LaiUhlrich2022_scaled.osim")

        let series = try MOTParser.parse(url: motionURL)
        let bridge = NimbleBridge()
        XCTAssertTrue(bridge.loadModel(fromPath: modelURL.path), "trail2 OpenCap model should load")

        let modelDOFs = bridge.dofNames as [String]
        let prepared = ImportedMotionPreparer.prepare(series: series, modelDOFNames: modelDOFs)

        XCTAssertEqual(series.frameCount, 3823)
        XCTAssertTrue(prepared.unmatchedSourceCoordinates.isEmpty, "trail2 .mot columns should all exist in the paired .osim")
        XCTAssertTrue(
            prepared.unmatchedModelDOFs.isEmpty,
            "NimbleBridge should expose a runtime DOF set that is fully matched by trail2 .mot coordinates"
        )
    }

    func testTrail2TRCMatchesMotionTimebase() throws {
        let datasetURL = try trail2DatasetURL()
        let motionURL = datasetURL.appendingPathComponent("run2.mot")
        let trcURL = datasetURL.appendingPathComponent("run2.trc")

        let motion = try MOTParser.parse(url: motionURL)
        let trc = try parseTRCSummary(url: trcURL)

        XCTAssertEqual(motion.frameCount, 3823)
        XCTAssertEqual(trc.numFrames, 3823)
        XCTAssertEqual(trc.dataRate, 120.0, accuracy: 1e-9)
        XCTAssertEqual(trc.cameraRate, 120.0, accuracy: 1e-9)
        XCTAssertEqual(trc.startTime, motion.frames.first?.timestamp ?? -1, accuracy: 1e-9)
        XCTAssertEqual(trc.endTime, motion.frames.last?.timestamp ?? -1, accuracy: 1e-4)
    }

    private func trail2DatasetURL(file: StaticString = #filePath) throws -> URL {
        let bundle = Bundle(for: type(of: self))
        if let bundledOSIM = bundle.url(forResource: "LaiUhlrich2022_scaled", withExtension: "osim", subdirectory: "trail2") {
            return bundledOSIM.deletingLastPathComponent()
        }

        let testFileURL = URL(fileURLWithPath: "\(file)")
        let repoRoot = testFileURL.deletingLastPathComponent().deletingLastPathComponent()
        let datasetURL = repoRoot.appendingPathComponent("demodata/opencap-demodata/trail2", isDirectory: true)
        guard FileManager.default.fileExists(atPath: datasetURL.path) else {
            throw XCTSkip("trail2 dataset not present at \(datasetURL.path)")
        }
        return datasetURL
    }

    private func parseTRCSummary(url: URL) throws -> (dataRate: Double, cameraRate: Double, numFrames: Int, startTime: Double, endTime: Double) {
        let lines = try String(contentsOf: url)
            .components(separatedBy: .newlines)
            .filter { !$0.isEmpty }

        XCTAssertGreaterThanOrEqual(lines.count, 7, "TRC file should contain metadata and data rows")
        let metadataValues = lines[2].split(whereSeparator: \.isWhitespace).map(String.init)
        XCTAssertGreaterThanOrEqual(metadataValues.count, 8, "TRC metadata row should contain expected fields")

        guard
            let dataRate = Double(metadataValues[0]),
            let cameraRate = Double(metadataValues[1]),
            let numFrames = Int(metadataValues[2])
        else {
            XCTFail("Unable to parse TRC metadata values")
            return (0, 0, 0, 0, 0)
        }

        let dataRows = Array(lines.dropFirst(5))
        XCTAssertEqual(dataRows.count, numFrames, "TRC row count should match metadata")

        func parseTime(_ row: String) throws -> Double {
            let columns = row.split(whereSeparator: \.isWhitespace).map(String.init)
            guard columns.count >= 2, let time = Double(columns[1]) else {
                throw NSError(domain: "ImportedMotionTests", code: 1, userInfo: [NSLocalizedDescriptionKey: "Malformed TRC row"])
            }
            return time
        }

        let startTime = try parseTime(dataRows.first ?? "")
        let endTime = try parseTime(dataRows.last ?? "")
        return (dataRate, cameraRate, numFrames, startTime, endTime)
    }
}
