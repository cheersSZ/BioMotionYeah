import Foundation

/// 9-point, cubic-order (P=3) Savitzky–Golay filter for online smoothing and
/// analytic differentiation of a scalar time series.
///
/// Each call to `push(_:timestamp:)` returns nil until 9 samples have been
/// accumulated. Once the window is full, it outputs the smoothed value,
/// first derivative, and second derivative evaluated at the CENTER of the
/// window — i.e. 4 samples behind the most recently pushed input.
///
/// At 60 fps this introduces ~66ms of lag, but the derivatives are far
/// cleaner than raw finite differences (which amplify position noise by
/// ~1/dt² for acceleration).
///
/// Coefficients are the closed-form solution of a local polynomial fit of
/// order P=3 to N=9 uniformly spaced samples. Verified against:
///   q(k) = k   → dq=1, ddq=0
///   q(k) = k²  → dq=0, ddq=2
final class SavitzkyGolayFilter {
    static let windowSize = 9
    static let halfWindow = 4

    // Position (0th derivative)
    private static let posCoeffs: [Double] =
        [-21.0, 14.0, 39.0, 54.0, 59.0, 54.0, 39.0, 14.0, -21.0].map { $0 / 231.0 }

    // Velocity (1st derivative, scale result by 1/dt)
    private static let velCoeffs: [Double] =
        [86.0, -142.0, -193.0, -126.0, 0.0, 126.0, 193.0, 142.0, -86.0].map { $0 / 1188.0 }

    // Acceleration (2nd derivative, scale result by 1/dt²)
    private static let accCoeffs: [Double] =
        [28.0, 7.0, -8.0, -17.0, -20.0, -17.0, -8.0, 7.0, 28.0].map { $0 / 462.0 }

    private var samples: [Double] = []
    private var timestamps: [Double] = []

    /// Push a new sample into the rolling window.
    /// Returns (smoothed position, velocity, acceleration, centerTimestamp) once
    /// the window is full, else nil during warm-up.
    func push(_ x: Double, timestamp: Double) -> (pos: Double, vel: Double, acc: Double, center: Double)? {
        samples.append(x)
        timestamps.append(timestamp)
        if samples.count > Self.windowSize {
            samples.removeFirst()
            timestamps.removeFirst()
        }
        guard samples.count == Self.windowSize else { return nil }

        // Effective sampling interval: window span divided by (N-1)
        let dt = (timestamps.last! - timestamps.first!) / Double(Self.windowSize - 1)
        guard dt > 1e-6 else { return nil }

        var pos = 0.0
        var vel = 0.0
        var acc = 0.0
        for i in 0..<Self.windowSize {
            let s = samples[i]
            pos += Self.posCoeffs[i] * s
            vel += Self.velCoeffs[i] * s
            acc += Self.accCoeffs[i] * s
        }
        return (pos, vel / dt, acc / (dt * dt), timestamps[Self.halfWindow])
    }

    func reset() {
        samples.removeAll(keepingCapacity: true)
        timestamps.removeAll(keepingCapacity: true)
    }

    var isWarmedUp: Bool { samples.count == Self.windowSize }
}
