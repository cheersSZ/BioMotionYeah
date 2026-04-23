import Foundation
import simd

// MARK: - Metadata schema

struct OfflineKeypointMetadata: Codable, Equatable {
    let format: String
    let video: OfflineVideoMetadata
    let keypointSet: OfflineKeypointSet
    let storage: OfflineKeypointStorage
}

struct OfflineVideoMetadata: Codable, Equatable {
    let filename: String
    let width: Int
    let height: Int
    let fps: Double
    let frameCount: Int
}

struct OfflineKeypointSet: Codable, Equatable {
    let name: String
    let points: [String]
}

struct OfflineKeypointStorage: Codable, Equatable {
    let binaryFilename: String
    let scalarType: String
    let channelsPerPoint: Int
    let channelOrder: [String]
    let layout: String
}

// MARK: - Single keypoint sample (image pixel space)

/// Image-space keypoint. Origin is top-left, `x` increases right, `y` increases down.
/// `confidence` is in [0, 1]; 0 means missing or below detector threshold.
struct OfflineKeypoint: Equatable {
    var x: Float
    var y: Float
    var confidence: Float

    var isPresent: Bool { confidence > 0 }
}

// MARK: - Loader

/// Memory-resident keypoint track backed by a frame-major float32 binary blob.
/// Decoded once at import time; sampling per playback frame is O(1) and allocation-free.
final class OfflineKeypointTrack {

    enum LoadError: Error, LocalizedError {
        case binaryNotFound(URL)
        case unsupportedFormat(String)
        case unsupportedScalarType(String)
        case unsupportedLayout(String)
        case unsupportedChannelLayout(channels: Int, order: [String])
        case binarySizeMismatch(expected: Int, actual: Int)
        case missingPointSet

        var errorDescription: String? {
            switch self {
            case .binaryNotFound(let url):
                return "Binary file not found next to metadata: \(url.lastPathComponent)"
            case .unsupportedFormat(let tag):
                return "Unsupported keypoint file format tag: \(tag)"
            case .unsupportedScalarType(let type):
                return "Unsupported keypoint scalar_type: \(type) (expected float32)"
            case .unsupportedLayout(let layout):
                return "Unsupported keypoint layout: \(layout) (expected frame-major)"
            case .unsupportedChannelLayout(let channels, let order):
                return "Unsupported channel layout: \(channels) channels [\(order.joined(separator: ", "))]"
            case .binarySizeMismatch(let expected, let actual):
                return "Keypoint binary size mismatch: expected \(expected) bytes, found \(actual)."
            case .missingPointSet:
                return "Keypoint metadata is missing the point name list."
            }
        }
    }

    static let supportedFormatTag = "biomotion-offline-keypoints-v1"

    let metadata: OfflineKeypointMetadata
    let pointsPerFrame: Int
    let frameCount: Int
    let metadataURL: URL
    let binaryURL: URL

    private let buffer: Data
    /// Bytes per frame: pointsPerFrame * 3 channels * 4 bytes.
    private let frameStride: Int

    private init(
        metadata: OfflineKeypointMetadata,
        buffer: Data,
        metadataURL: URL,
        binaryURL: URL
    ) {
        self.metadata = metadata
        self.pointsPerFrame = metadata.keypointSet.points.count
        self.frameStride = pointsPerFrame * 3 * MemoryLayout<Float32>.size
        self.frameCount = buffer.count / max(self.frameStride, 1)
        self.buffer = buffer
        self.metadataURL = metadataURL
        self.binaryURL = binaryURL
    }

    static func load(metadataURL: URL, binaryURL: URL? = nil) throws -> OfflineKeypointTrack {
        let metadataData = try Data(contentsOf: metadataURL)
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        let metadata = try decoder.decode(OfflineKeypointMetadata.self, from: metadataData)

        guard metadata.format == supportedFormatTag else {
            throw LoadError.unsupportedFormat(metadata.format)
        }
        guard !metadata.keypointSet.points.isEmpty else {
            throw LoadError.missingPointSet
        }
        guard metadata.storage.scalarType.lowercased() == "float32" else {
            throw LoadError.unsupportedScalarType(metadata.storage.scalarType)
        }
        guard metadata.storage.layout.lowercased() == "frame-major" else {
            throw LoadError.unsupportedLayout(metadata.storage.layout)
        }
        guard metadata.storage.channelsPerPoint == 3,
              metadata.storage.channelOrder.map({ $0.lowercased() }) == ["x", "y", "confidence"] else {
            throw LoadError.unsupportedChannelLayout(
                channels: metadata.storage.channelsPerPoint,
                order: metadata.storage.channelOrder
            )
        }

        let resolvedBinaryURL: URL
        if let binaryURL {
            resolvedBinaryURL = binaryURL
        } else {
            resolvedBinaryURL = metadataURL
                .deletingLastPathComponent()
                .appendingPathComponent(metadata.storage.binaryFilename)
        }

        guard FileManager.default.fileExists(atPath: resolvedBinaryURL.path) else {
            throw LoadError.binaryNotFound(resolvedBinaryURL)
        }

        let buffer = try Data(contentsOf: resolvedBinaryURL, options: .mappedIfSafe)

        let pointsPerFrame = metadata.keypointSet.points.count
        let frameStride = pointsPerFrame * 3 * MemoryLayout<Float32>.size
        let expectedBytes = metadata.video.frameCount * frameStride

        if buffer.count != expectedBytes {
            // Be permissive: if the buffer is a whole number of frames we
            // accept it, but anything else is a hard error.
            if buffer.count % frameStride != 0 {
                throw LoadError.binarySizeMismatch(expected: expectedBytes, actual: buffer.count)
            }
        }

        return OfflineKeypointTrack(
            metadata: metadata,
            buffer: buffer,
            metadataURL: metadataURL,
            binaryURL: resolvedBinaryURL
        )
    }

    var fps: Double { metadata.video.fps }
    var pixelWidth: Int { metadata.video.width }
    var pixelHeight: Int { metadata.video.height }
    var pointNames: [String] { metadata.keypointSet.points }
    var keypointSetName: String { metadata.keypointSet.name }

    /// Frame index closest to `time` (seconds), clamped to [0, frameCount-1].
    func frameIndex(for time: TimeInterval) -> Int {
        guard frameCount > 0 else { return 0 }
        if !time.isFinite || time <= 0 { return 0 }
        let raw = Int((time * fps).rounded())
        return min(max(raw, 0), frameCount - 1)
    }

    /// Returns the keypoints for the given frame index, or nil when out of range.
    func frame(at index: Int) -> [OfflineKeypoint]? {
        guard (0..<frameCount).contains(index) else { return nil }
        let start = index * frameStride
        var out: [OfflineKeypoint] = []
        out.reserveCapacity(pointsPerFrame)
        buffer.withUnsafeBytes { rawBytes in
            let base = rawBytes.baseAddress!.advanced(by: start).assumingMemoryBound(to: Float32.self)
            for p in 0..<pointsPerFrame {
                let ptr = base.advanced(by: p * 3)
                out.append(OfflineKeypoint(x: ptr[0], y: ptr[1], confidence: ptr[2]))
            }
        }
        return out
    }

    /// Returns the keypoints sampled at playback `time` (seconds).
    func frame(atTime time: TimeInterval) -> [OfflineKeypoint]? {
        frame(at: frameIndex(for: time))
    }
}

// MARK: - OpenPose-25 skeleton edges

/// Bone topology for the OpenPose-25 / OpenCap layout used by the preprocessor.
/// Indices reference the `OfflineKeypointMetadata.keypointSet.points` array order.
enum OpenPose25Skeleton {

    static let pointCount = 25

    /// (parent, child) pairs that define the bone segments to draw.
    static let edges: [(Int, Int)] = [
        (0, 1),    // Nose -> Neck
        (1, 2),    // Neck -> RShoulder
        (2, 3),    // RShoulder -> RElbow
        (3, 4),    // RElbow -> RWrist
        (1, 5),    // Neck -> LShoulder
        (5, 6),    // LShoulder -> LElbow
        (6, 7),    // LElbow -> LWrist
        (1, 8),    // Neck -> midHip
        (8, 9),    // midHip -> RHip
        (9, 10),   // RHip -> RKnee
        (10, 11),  // RKnee -> RAnkle
        (8, 12),   // midHip -> LHip
        (12, 13),  // LHip -> LKnee
        (13, 14),  // LKnee -> LAnkle
        (0, 15),   // Nose -> REye
        (15, 17),  // REye -> REar
        (0, 16),   // Nose -> LEye
        (16, 18),  // LEye -> LEar
        (14, 19),  // LAnkle -> LBigToe
        (19, 20),  // LBigToe -> LSmallToe
        (14, 21),  // LAnkle -> LHeel
        (11, 22),  // RAnkle -> RBigToe
        (22, 23),  // RBigToe -> RSmallToe
        (11, 24)   // RAnkle -> RHeel
    ]

    enum Side { case center, left, right }

    /// Side classification for color-coding bones (left = warm, right = cool, center = neutral).
    static func side(forEdge edge: (Int, Int)) -> Side {
        let leftIndices: Set<Int> = [5, 6, 7, 12, 13, 14, 16, 18, 19, 20, 21]
        let rightIndices: Set<Int> = [2, 3, 4, 9, 10, 11, 15, 17, 22, 23, 24]
        let containsLeft = leftIndices.contains(edge.0) || leftIndices.contains(edge.1)
        let containsRight = rightIndices.contains(edge.0) || rightIndices.contains(edge.1)
        if containsLeft && !containsRight { return .left }
        if containsRight && !containsLeft { return .right }
        return .center
    }

    static func side(forPointIndex index: Int) -> Side {
        let leftIndices: Set<Int> = [5, 6, 7, 12, 13, 14, 16, 18, 19, 20, 21]
        let rightIndices: Set<Int> = [2, 3, 4, 9, 10, 11, 15, 17, 22, 23, 24]
        if leftIndices.contains(index) { return .left }
        if rightIndices.contains(index) { return .right }
        return .center
    }
}

// MARK: - Pixel → view-space mapping helper

/// Maps the keypoint track's pixel space onto an aspect-fit display rect.
struct AspectFitMapping {
    let displayRect: CGRect
    let pixelSize: CGSize

    init(pixelSize: CGSize, in container: CGSize) {
        self.pixelSize = pixelSize
        if pixelSize.width <= 0 || pixelSize.height <= 0 || container.width <= 0 || container.height <= 0 {
            self.displayRect = .zero
            return
        }
        let pixelAspect = pixelSize.width / pixelSize.height
        let viewAspect = container.width / container.height
        let displayWidth: CGFloat
        let displayHeight: CGFloat
        if pixelAspect > viewAspect {
            displayWidth = container.width
            displayHeight = container.width / pixelAspect
        } else {
            displayHeight = container.height
            displayWidth = container.height * pixelAspect
        }
        let originX = (container.width - displayWidth) * 0.5
        let originY = (container.height - displayHeight) * 0.5
        self.displayRect = CGRect(x: originX, y: originY, width: displayWidth, height: displayHeight)
    }

    func point(forPixel x: CGFloat, _ y: CGFloat) -> CGPoint {
        guard pixelSize.width > 0, pixelSize.height > 0, displayRect.width > 0, displayRect.height > 0 else {
            return .zero
        }
        let nx = x / pixelSize.width
        let ny = y / pixelSize.height
        return CGPoint(
            x: displayRect.minX + nx * displayRect.width,
            y: displayRect.minY + ny * displayRect.height
        )
    }
}
