import AVKit
import SwiftUI
import UIKit

/// SwiftUI wrapper around `AVPlayerLayer` with no built-in playback controls.
/// Used by the offline mode so a custom 2D pose overlay can be aligned over the
/// rendered video frame without fighting AVKit's chrome.
struct PlayerLayerView: UIViewRepresentable {
    let player: AVPlayer
    var videoGravity: AVLayerVideoGravity = .resizeAspect

    func makeUIView(context: Context) -> PlayerContainerView {
        let view = PlayerContainerView()
        view.playerLayer.player = player
        view.playerLayer.videoGravity = videoGravity
        view.backgroundColor = .black
        view.isUserInteractionEnabled = false
        return view
    }

    func updateUIView(_ uiView: PlayerContainerView, context: Context) {
        if uiView.playerLayer.player !== player {
            uiView.playerLayer.player = player
        }
        if uiView.playerLayer.videoGravity != videoGravity {
            uiView.playerLayer.videoGravity = videoGravity
        }
    }

    final class PlayerContainerView: UIView {
        override class var layerClass: AnyClass { AVPlayerLayer.self }

        var playerLayer: AVPlayerLayer { layer as! AVPlayerLayer }
    }
}
