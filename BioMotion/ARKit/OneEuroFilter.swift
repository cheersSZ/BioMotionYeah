import simd

/// Adaptive low-pass filter for smoothing noisy signals in real-time.
/// Ref: Casiez et al., "1-Euro Filter: A Simple Speed-based Low-pass Filter
/// for Noisy Input in Interactive Systems" (CHI 2012).
///
/// Key property: filters slow movements heavily (remove jitter) while
/// passing fast movements with minimal lag.
final class OneEuroFilter3D {
    private var xFilter: OneEuroFilter
    private var yFilter: OneEuroFilter
    private var zFilter: OneEuroFilter

    /// - Parameters:
    ///   - minCutoff: Minimum cutoff frequency in Hz. Lower = more smoothing at rest.
    ///                Default 1.0 Hz works well for body tracking.
    ///   - beta: Speed coefficient. Higher = less lag during fast movement.
    ///           Default 0.007 balances smoothness and responsiveness.
    ///   - dCutoff: Cutoff frequency for derivative estimation. Default 1.0 Hz.
    init(minCutoff: Double = 1.0, beta: Double = 0.007, dCutoff: Double = 1.0) {
        xFilter = OneEuroFilter(minCutoff: minCutoff, beta: beta, dCutoff: dCutoff)
        yFilter = OneEuroFilter(minCutoff: minCutoff, beta: beta, dCutoff: dCutoff)
        zFilter = OneEuroFilter(minCutoff: minCutoff, beta: beta, dCutoff: dCutoff)
    }

    func filter(_ value: SIMD3<Float>, timestamp: TimeInterval) -> SIMD3<Float> {
        SIMD3<Float>(
            Float(xFilter.filter(Double(value.x), timestamp: timestamp)),
            Float(yFilter.filter(Double(value.y), timestamp: timestamp)),
            Float(zFilter.filter(Double(value.z), timestamp: timestamp))
        )
    }

    func reset() {
        xFilter.reset()
        yFilter.reset()
        zFilter.reset()
    }
}

/// 1-Euro filter for a single scalar value.
final class OneEuroFilter {
    private let minCutoff: Double
    private let beta: Double
    private let dCutoff: Double

    private var xPrev: Double?
    private var dxPrev: Double = 0
    private var tPrev: Double?

    init(minCutoff: Double = 1.0, beta: Double = 0.007, dCutoff: Double = 1.0) {
        self.minCutoff = minCutoff
        self.beta = beta
        self.dCutoff = dCutoff
    }

    func filter(_ x: Double, timestamp: Double) -> Double {
        guard let xPrev, let tPrev else {
            // First sample — no filtering
            self.xPrev = x
            self.tPrev = timestamp
            return x
        }

        let dt = timestamp - tPrev
        guard dt > 0 else { return xPrev }

        let rate = 1.0 / dt

        // Estimate derivative
        let dx = (x - xPrev) * rate
        let edx = smoothingFactor(rate: rate, cutoff: dCutoff)
        let dxFiltered = edx * dx + (1 - edx) * dxPrev

        // Adaptive cutoff based on speed
        let cutoff = minCutoff + beta * abs(dxFiltered)

        // Filter the signal
        let alpha = smoothingFactor(rate: rate, cutoff: cutoff)
        let xFiltered = alpha * x + (1 - alpha) * xPrev

        self.xPrev = xFiltered
        self.dxPrev = dxFiltered
        self.tPrev = timestamp

        return xFiltered
    }

    func reset() {
        xPrev = nil
        dxPrev = 0
        tPrev = nil
    }

    private func smoothingFactor(rate: Double, cutoff: Double) -> Double {
        let tau = 1.0 / (2.0 * .pi * cutoff)
        let te = 1.0 / rate
        return 1.0 / (1.0 + tau / te)
    }
}

/// Manages per-joint 1-euro filters for the entire skeleton.
final class SkeletonFilter {
    private var filters: [String: OneEuroFilter3D] = [:]
    private let minCutoff: Double
    private let beta: Double

    /// - Parameters:
    ///   - minCutoff: Minimum cutoff frequency. Default 1.0 Hz.
    ///   - beta: Speed coefficient. Default 0.007.
    init(minCutoff: Double = 1.0, beta: Double = 0.007) {
        self.minCutoff = minCutoff
        self.beta = beta
    }

    /// Filter all joints in a body frame, returning a new frame with smoothed positions.
    func filter(_ frame: BodyFrame) -> BodyFrame {
        let filteredJoints = frame.joints.map { joint -> TrackedJoint in
            guard joint.isTracked else { return joint }

            let filter = filters[joint.id] ?? {
                let f = OneEuroFilter3D(minCutoff: minCutoff, beta: beta)
                filters[joint.id] = f
                return f
            }()

            let smoothed = filter.filter(joint.worldPosition, timestamp: frame.timestamp)
            return TrackedJoint(
                id: joint.id,
                name: joint.name,
                worldPosition: smoothed,
                isTracked: joint.isTracked
            )
        }

        return BodyFrame(
            timestamp: frame.timestamp,
            frameNumber: frame.frameNumber,
            joints: filteredJoints
        )
    }

    func reset() {
        filters.removeAll()
    }
}
