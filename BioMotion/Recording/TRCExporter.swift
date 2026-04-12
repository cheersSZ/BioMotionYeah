import Foundation

/// Exports recorded BodyFrames to OpenSim .trc format.
/// TRC (Track Row Column) is a tab-delimited format storing 3D marker positions over time.
struct TRCExporter {
    let frames: [BodyFrame]
    let markerMappings: [JointMapping.Mapping]

    init(frames: [BodyFrame], markerMappings: [JointMapping.Mapping] = JointMapping.primary) {
        self.frames = frames
        self.markerMappings = markerMappings
    }

    /// Generates a .trc file string compatible with OpenSim.
    func generate() -> String {
        guard let firstFrame = frames.first else { return "" }

        let numFrames = frames.count
        let numMarkers = markerMappings.count
        let dataRate = calculateDataRate()

        var lines: [String] = []

        // Line 1: PathFileType header
        lines.append("PathFileType\t4\t(X/Y/Z)\tBioMotion_capture.trc")

        // Line 2: Metadata keys
        lines.append("DataRate\tCameraRate\tNumFrames\tNumMarkers\tUnits\tOrigDataRate\tOrigDataStartFrame\tOrigNumFrames")

        // Line 3: Metadata values
        lines.append(String(format: "%.2f\t%.2f\t%d\t%d\tm\t%.2f\t1\t%d",
                            dataRate, dataRate, numFrames, numMarkers, dataRate, numFrames))

        // Line 4: Marker names (each followed by two empty tabs for Y/Z columns)
        var markerLine = "Frame#\tTime"
        for mapping in markerMappings {
            markerLine += "\t\(mapping.opensimName)\t\t"
        }
        lines.append(markerLine)

        // Line 5: Component labels (X/Y/Z per marker)
        var componentLine = "\t"
        for i in 1...numMarkers {
            componentLine += "\tX\(i)\tY\(i)\tZ\(i)"
        }
        lines.append(componentLine)

        // Line 6: Blank line
        lines.append("")

        // Data rows
        let startTime = firstFrame.timestamp
        for frame in frames {
            let time = frame.timestamp - startTime
            var row = "\(frame.frameNumber)\t\(String(format: "%.6f", time))"

            for mapping in markerMappings {
                if let joint = frame.joints.first(where: { $0.id == mapping.arkitName }),
                   joint.isTracked {
                    row += String(format: "\t%.6f\t%.6f\t%.6f",
                                  joint.worldPosition.x,
                                  joint.worldPosition.y,
                                  joint.worldPosition.z)
                } else {
                    // Missing marker: use NaN (OpenSim convention)
                    row += "\t\t\t"
                }
            }

            lines.append(row)
        }

        return lines.joined(separator: "\n")
    }

    /// Writes the .trc file to a temporary location and returns the URL.
    func export(filename: String = "BioMotion_capture") throws -> URL {
        let content = generate()
        let url = FileManager.default.temporaryDirectory
            .appendingPathComponent("\(filename).trc")
        try content.write(to: url, atomically: true, encoding: .utf8)
        return url
    }

    private func calculateDataRate() -> Double {
        guard frames.count > 1,
              let first = frames.first,
              let last = frames.last else { return 60.0 }
        let elapsed = last.timestamp - first.timestamp
        guard elapsed > 0 else { return 60.0 }
        return Double(frames.count - 1) / elapsed
    }
}
