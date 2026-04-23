import SwiftUI

/// Draws the OpenPose-25 skeleton on top of the offline video frame.
///
/// The view is purely visual; it expects `pixelSize` to match the keypoint
/// track's metadata so the aspect-fit transform lands the bones on the
/// underlying `AVPlayerLayer.resizeAspect` content rect.
struct PoseOverlayView: View {

    let keypoints: [OfflineKeypoint]
    let pixelSize: CGSize
    var confidenceThreshold: Float = 0.2
    var jointRadius: CGFloat = 4
    var boneWidth: CGFloat = 3

    var body: some View {
        GeometryReader { geometry in
            Canvas { context, size in
                draw(in: &context, size: size)
            }
            .frame(width: geometry.size.width, height: geometry.size.height)
            .allowsHitTesting(false)
        }
    }

    private func draw(in context: inout GraphicsContext, size: CGSize) {
        guard !keypoints.isEmpty,
              pixelSize.width > 0, pixelSize.height > 0,
              size.width > 0, size.height > 0 else { return }

        let mapping = AspectFitMapping(pixelSize: pixelSize, in: size)
        guard mapping.displayRect.width > 0 else { return }

        // Bones first so dots paint over the line endpoints.
        for edge in OpenPose25Skeleton.edges where edge.0 < keypoints.count && edge.1 < keypoints.count {
            let a = keypoints[edge.0]
            let b = keypoints[edge.1]
            guard a.confidence >= confidenceThreshold, b.confidence >= confidenceThreshold else { continue }

            let pa = mapping.point(forPixel: CGFloat(a.x), CGFloat(a.y))
            let pb = mapping.point(forPixel: CGFloat(b.x), CGFloat(b.y))
            var path = Path()
            path.move(to: pa)
            path.addLine(to: pb)

            let alpha = Double(min(a.confidence, b.confidence))
            let color = OpenPose25Skeleton.boneColor(for: edge).opacity(0.55 + 0.45 * alpha)
            context.stroke(path, with: .color(color), lineWidth: boneWidth)
        }

        for (index, point) in keypoints.enumerated() where point.confidence >= confidenceThreshold {
            let p = mapping.point(forPixel: CGFloat(point.x), CGFloat(point.y))
            let rect = CGRect(
                x: p.x - jointRadius,
                y: p.y - jointRadius,
                width: jointRadius * 2,
                height: jointRadius * 2
            )
            let color = OpenPose25Skeleton.jointColor(for: index)
                .opacity(0.6 + 0.4 * Double(point.confidence))
            context.fill(Path(ellipseIn: rect), with: .color(color))
        }
    }
}

private extension OpenPose25Skeleton {
    static func boneColor(for edge: (Int, Int)) -> Color {
        switch side(forEdge: edge) {
        case .left: return .orange
        case .right: return .cyan
        case .center: return .green
        }
    }

    static func jointColor(for index: Int) -> Color {
        switch side(forPointIndex: index) {
        case .left: return .yellow
        case .right: return .blue
        case .center: return .white
        }
    }
}
