import Foundation

struct ImportedMotionSeries {
    let sourceURL: URL?
    let coordinateNames: [String]
    let frames: [ImportedMotionFrame]
    let rotationalValuesAreDegrees: Bool

    var frameCount: Int { frames.count }
}

struct ImportedMotionFrame {
    let index: Int
    let timestamp: TimeInterval
    let coordinates: [String: Double]
}

struct PreparedMotionFrame {
    let index: Int
    let timestamp: TimeInterval
    let modelDOFNames: [String]
    let positions: [Double]
}

struct ImportedMotionPreparation {
    let modelDOFNames: [String]
    let frames: [PreparedMotionFrame]
    let unmatchedSourceCoordinates: [String]
    let unmatchedModelDOFs: [String]
}

enum MOTParser {
    enum ParseError: Error, LocalizedError {
        case invalidHeader
        case missingEndHeader
        case missingColumnHeader
        case malformedRow(line: String)

        var errorDescription: String? {
            switch self {
            case .invalidHeader:
                return "The .mot file header is invalid."
            case .missingEndHeader:
                return "The .mot file is missing an endheader line."
            case .missingColumnHeader:
                return "The .mot file is missing the time/coordinate header row."
            case .malformedRow(let line):
                return "Malformed .mot row: \(line)"
            }
        }
    }

    static func parse(url: URL) throws -> ImportedMotionSeries {
        try parse(text: String(contentsOf: url), sourceURL: url)
    }

    static func parse(text: String, sourceURL: URL? = nil) throws -> ImportedMotionSeries {
        let lines = text
            .components(separatedBy: .newlines)
            .map { $0.trimmingCharacters(in: .whitespacesAndNewlines) }
            .filter { !$0.isEmpty }

        guard !lines.isEmpty else { throw ParseError.invalidHeader }
        guard let endHeaderIndex = lines.firstIndex(where: { $0.lowercased() == "endheader" }) else {
            throw ParseError.missingEndHeader
        }

        let headerLines = Array(lines[..<endHeaderIndex])
        let dataLines = Array(lines[(endHeaderIndex + 1)...])
        guard let columnLine = dataLines.first else { throw ParseError.missingColumnHeader }

        let rotationalValuesAreDegrees = headerLines.contains { $0.lowercased() == "indegrees=yes" }
        let columns = splitColumns(columnLine)
        guard columns.count >= 2, columns[0] == "time" else { throw ParseError.missingColumnHeader }
        let coordinateNames = Array(columns.dropFirst())

        var frames: [ImportedMotionFrame] = []
        frames.reserveCapacity(max(0, dataLines.count - 1))

        for (index, line) in dataLines.dropFirst().enumerated() {
            let columns = splitColumns(line)
            guard columns.count == coordinateNames.count + 1 else {
                throw ParseError.malformedRow(line: line)
            }
            guard let time = Double(columns[0]) else {
                throw ParseError.malformedRow(line: line)
            }

            var coordinates: [String: Double] = [:]
            coordinates.reserveCapacity(coordinateNames.count)
            for (name, valueString) in zip(coordinateNames, columns.dropFirst()) {
                guard let value = Double(valueString) else {
                    throw ParseError.malformedRow(line: line)
                }
                coordinates[name] = value
            }
            frames.append(ImportedMotionFrame(index: index, timestamp: time, coordinates: coordinates))
        }

        return ImportedMotionSeries(
            sourceURL: sourceURL,
            coordinateNames: coordinateNames,
            frames: frames,
            rotationalValuesAreDegrees: rotationalValuesAreDegrees
        )
    }

    private static func splitColumns(_ line: String) -> [String] {
        line.split(whereSeparator: \.isWhitespace).map(String.init)
    }
}

enum ImportedMotionPreparer {
    static func prepare(
        series: ImportedMotionSeries,
        modelDOFNames: [String]
    ) -> ImportedMotionPreparation {
        let aliasMap = coordinateAliasMap()
        let normalizedModelDOFs = Set(modelDOFNames)

        var sourceToModel: [String: String] = [:]
        var matchedModelDOFs = Set<String>()

        for sourceName in series.coordinateNames {
            if normalizedModelDOFs.contains(sourceName) {
                sourceToModel[sourceName] = sourceName
                matchedModelDOFs.insert(sourceName)
                continue
            }
            if let alias = aliasMap[sourceName], normalizedModelDOFs.contains(alias) {
                sourceToModel[sourceName] = alias
                matchedModelDOFs.insert(alias)
            }
        }

        let unmatchedSourceCoordinates = series.coordinateNames.filter { sourceToModel[$0] == nil }
        let unmatchedModelDOFs = modelDOFNames.filter { !matchedModelDOFs.contains($0) }

        let frames = series.frames.map { frame in
            let positions = modelDOFNames.map { modelDOF -> Double in
                let sourceName = sourceToModel.first(where: { $0.value == modelDOF })?.key
                guard let sourceName, let rawValue = frame.coordinates[sourceName] else { return 0.0 }
                return convert(rawValue, sourceName: sourceName, rotationalValuesAreDegrees: series.rotationalValuesAreDegrees)
            }
            return PreparedMotionFrame(
                index: frame.index,
                timestamp: frame.timestamp,
                modelDOFNames: modelDOFNames,
                positions: positions
            )
        }

        return ImportedMotionPreparation(
            modelDOFNames: modelDOFNames,
            frames: frames,
            unmatchedSourceCoordinates: unmatchedSourceCoordinates,
            unmatchedModelDOFs: unmatchedModelDOFs
        )
    }

    private static func convert(_ rawValue: Double, sourceName: String, rotationalValuesAreDegrees: Bool) -> Double {
        guard rotationalValuesAreDegrees, !isTranslationCoordinate(sourceName) else { return rawValue }
        return rawValue * .pi / 180.0
    }

    private static func isTranslationCoordinate(_ name: String) -> Bool {
        name.hasSuffix("_tx") || name.hasSuffix("_ty") || name.hasSuffix("_tz")
    }

    private static func coordinateAliasMap() -> [String: String] {
        [
            "lumbar_extension": "Abs_FE",
            "lumbar_bending": "Abs_LB",
            "lumbar_rotation": "Abs_AR"
        ]
    }
}
