import Foundation
import Combine
import QuartzCore

/// Manages the Nimble physics engine lifecycle and provides real-time IK/ID results.
/// Runs Nimble on a background queue to avoid blocking the main thread.
final class NimbleEngine: ObservableObject {
    @Published var isModelLoaded = false
    @Published var lastIKResult: IKOutput?
    @Published var lastIDResult: IDOutput?
    @Published var lastMuscleResult: MuscleOutput?
    @Published var ikSolveTimeMs: Double = 0
    @Published var idSolveTimeMs: Double = 0
    @Published var muscleSolveTimeMs: Double = 0

    // --- Accuracy metrics (for UI diagnostics) ---
    /// RMS marker residual from the most recent IK solve, in meters.
    @Published var ikMarkerResidualMeters: Double = 0
    /// Max |joint torque| / total body mass from the most recent ID solve, in Nm/kg.
    /// Physiological range for walking/squat: ~1–3 Nm/kg. Values above 10 indicate
    /// broken pipeline (usually missing GRF or bad ddq).
    @Published var maxTorquePerKg: Double = 0
    /// Total mass of the loaded (possibly scaled) skeleton, in kg.
    @Published var totalMassKg: Double = 0
    /// Left/right foot vertical GRF as fractions of body weight (0-1.x typically).
    /// Sum should be ~1.0 in steady stance, 0 in flight, 1.0-3.0 during impact.
    @Published var leftFootLoadFraction: Double = 0
    @Published var rightFootLoadFraction: Double = 0
    /// Root 6D residual after GRF solve. < ~0.5 Nm/kg means GRF is consistent.
    @Published var rootResidualPerKg: Double = 0
    /// Current ground-plane height (ARKit world y), for display only.
    @Published var groundHeightY: Double = 0

    /// Processed IK output with named DOFs.
    struct IKOutput {
        let jointAngles: [String: Double]  // DOF name → angle in radians
        let error: Double                   // RMS marker error in meters
        let timestamp: TimeInterval
    }

    /// Processed ID output with named DOFs.
    struct IDOutput {
        let jointTorques: [String: Double]  // DOF name → torque in Nm
        let timestamp: TimeInterval

        // Ground-reaction-force diagnostics. Forces in newtons (world frame),
        // CoPs in meters (world frame). Zero when the foot is not in contact.
        var leftFootForce: SIMD3<Double> = .zero
        var rightFootForce: SIMD3<Double> = .zero
        var leftFootCoP: SIMD3<Double> = .zero
        var rightFootCoP: SIMD3<Double> = .zero
        var leftFootInContact: Bool = false
        var rightFootInContact: Bool = false
        /// Norm of the 6D residual at the floating root joint. Should be
        /// small (< ~10 Nm) when GRF and kinematics are consistent.
        var rootResidualNorm: Double = 0
    }

    /// Processed muscle optimization output.
    struct MuscleOutput {
        let activations: [String: Double]  // muscle name → activation 0-1
        let forces: [String: Double]       // muscle name → force in N
        let converged: Bool
        let timestamp: TimeInterval
    }

    private let bridge = NimbleBridge()
    private let muscleSolver = MuscleSolver()
    private let momentArmComputer = MomentArmComputer()
    private let solverQueue = DispatchQueue(label: "com.biomotion.nimble", qos: .userInteractive)

    // Per-DOF Savitzky–Golay filters for smoothed q / dq / ddq.
    // Warms up after 9 frames (~150 ms @ 60 fps); once warm, outputs lag
    // the raw input by 4 frames (~66 ms @ 60 fps) in exchange for much
    // cleaner numerical derivatives than naive finite differences.
    private var dofFilters: [SavitzkyGolayFilter] = []

    // Timestamp of the last successful muscle solve, used to derive dt for
    // musculotendon length finite differencing inside the muscle Hill model.
    private var lastMuscleSolveTimestamp: TimeInterval?

    // IK history for recording
    private(set) var ikHistory: [(timestamp: TimeInterval, angles: [String: Double], error: Double)] = []
    private(set) var idHistory: [(timestamp: TimeInterval, jointTorques: [String: Double])] = []
    private var isRecordingResults = false

    /// Load the bundled .osim model.
    func loadBundledModel() {
        // Production full-body model: cyclistFullBodyMuscle.osim shipped
        // as FullBody.osim in the app bundle. 80 bodies, 520 muscles
        // (Millard2012 lower + Thelen2003 upper/trunk/spine).
        //
        // The libnimble_ios.a in this build includes a patched
        // OpenSimParser that no longer crashes on CustomJoints it can't
        // construct — it logs them and substitutes a WeldJoint. So any
        // joint in cyclist that nimble doesn't fully support becomes
        // locked rather than segfaulting the whole skeleton.
        //
        // Fallback: if FullBody.osim is missing from the bundle, load
        // the old lower-extremity-only Rajagopal2016 — useful for
        // developers who want to diff behavior without re-ripping assets.
        let path: String
        if let fb = Bundle.main.path(forResource: "FullBody", ofType: "osim") {
            path = fb
            print("NimbleEngine: loading FullBody.osim (cyclistFullBodyMuscle — 520 muscles, nimble OpenSimParser patched)")
        } else if let raj = Bundle.main.path(forResource: "Rajagopal2016", ofType: "osim") {
            path = raj
            print("NimbleEngine: ⚠ FullBody.osim not found — falling back to Rajagopal2016 (81 lower-extremity muscles only)")
        } else {
            print("NimbleEngine: no .osim model found in bundle")
            return
        }
        solverQueue.async { [weak self] in
            guard let self else { return }
            let success = self.bridge.loadModel(fromPath: path)
            if success {
                self.muscleSolver.loadMuscles(fromOsimPath: path)
                // MomentArmComputer adopts the bridge's skeleton instead of
                // parsing a second copy — so per-segment scaling propagates
                // from bridge.scaleModelWithHeight through to R(q) and L_MT.
                self.momentArmComputer.parseMusclePaths(fromOsimPath: path,
                                                         from: self.bridge)
                // Drop any stale SG state from a previous model — the new
                // model may have a different DOF count / ordering, and even
                // if not, the sample history is no longer valid.
                self.dofFilters.removeAll(keepingCapacity: false)
            }
            DispatchQueue.main.async {
                self.isModelLoaded = success
                if success {
                    let modelMarkers = self.bridge.markerNames as [String]
                    self.totalMassKg = self.bridge.totalMass
                    print("NimbleEngine: Model loaded — \(self.bridge.numDOFs) DOFs, \(modelMarkers.count) markers, \(self.muscleSolver.numMuscles) muscles, mass \(String(format: "%.1f", self.totalMassKg)) kg")
                    print("NimbleEngine: Model marker names (first 10): \(modelMarkers.prefix(10))")
                    print("NimbleEngine: Our mapping names (first 10): \(JointMapping.primary.map(\.opensimName).prefix(10))")
                    // Check how many of our mappings match model markers
                    let matchCount = JointMapping.primary.filter { modelMarkers.contains($0.opensimName) }.count
                    print("NimbleEngine: Marker matches: \(matchCount)/\(JointMapping.primary.count)")
                }
            }
        }
    }

    /// Scale the model for a specific user.
    func scaleModel(height: Double, markerPositions: [Float], markerNames: [String]) {
        guard isModelLoaded else { return }
        let positions = markerPositions.map { NSNumber(value: Double($0)) }
        solverQueue.async { [weak self] in
            self?.bridge.scaleModel(withHeight: height,
                                    markerPositions: positions,
                                    markerNames: markerNames)
        }
    }

    /// Process a body frame: run IK (and optionally ID) on a background thread.
    func processFrame(_ frame: BodyFrame) {
        guard isModelLoaded else { return }

        // Build marker arrays from the frame
        var positions: [NSNumber] = []
        var names: [String] = []

        for joint in frame.joints where joint.isTracked {
            // Map ARKit joint to OpenSim marker name
            if let mapping = JointMapping.primary.first(where: { $0.arkitName == joint.id }) {
                names.append(mapping.opensimName)
                positions.append(NSNumber(value: Double(joint.worldPosition.x)))
                positions.append(NSNumber(value: Double(joint.worldPosition.y)))
                positions.append(NSNumber(value: Double(joint.worldPosition.z)))
            }
        }

        guard !names.isEmpty else { return }

        solverQueue.async { [weak self] in
            guard let self else { return }

            // --- IK (runs on every frame, on 1€-filtered markers) ---
            let ikStart = CACurrentMediaTime()
            guard let ikResult = self.bridge.solveIK(
                withMarkerPositions: positions,
                markerNames: names
            ) else { return }
            let ikTime = (CACurrentMediaTime() - ikStart) * 1000.0

            let numDOFs = ikResult.jointAngles.count
            let dofNames = ikResult.dofNames

            // Lazy-init per-DOF SG filters whenever the DOF count changes.
            if self.dofFilters.count != numDOFs {
                self.dofFilters = (0..<numDOFs).map { _ in SavitzkyGolayFilter() }
            }

            // --- Push raw IK angles into SG filters, pull smoothed q/dq/ddq ---
            // Each filter only emits output once its 9-sample window is full,
            // so the first 8 frames after start (or after a DOF-count change)
            // yield "raw" IK only, without ID or muscle results.
            var smoothedQ = [Double](); smoothedQ.reserveCapacity(numDOFs)
            var smoothedDQ = [Double](); smoothedDQ.reserveCapacity(numDOFs)
            var smoothedDDQ = [Double](); smoothedDDQ.reserveCapacity(numDOFs)
            var centerTimestamp = frame.timestamp
            var sgWarmedUp = true

            for i in 0..<numDOFs {
                let q = ikResult.jointAngles[i].doubleValue
                if let out = self.dofFilters[i].push(q, timestamp: frame.timestamp) {
                    smoothedQ.append(out.pos)
                    smoothedDQ.append(out.vel)
                    smoothedDDQ.append(out.acc)
                    centerTimestamp = out.center
                } else {
                    sgWarmedUp = false
                    break
                }
            }

            // Build the "raw" IK output (used for live UI skeleton overlay).
            var liveAngles: [String: Double] = [:]
            for i in 0..<min(numDOFs, dofNames.count) {
                liveAngles[dofNames[i]] = ikResult.jointAngles[i].doubleValue
            }
            let liveIkOutput = IKOutput(
                jointAngles: liveAngles,
                error: ikResult.error,
                timestamp: frame.timestamp
            )

            guard sgWarmedUp else {
                // Still warming up: publish live IK only, no ID/muscle yet.
                self.publishResults(ik: liveIkOutput, id: nil, muscle: nil,
                                    ikTime: ikTime, idTime: 0, muscleTime: 0,
                                    ikResidual: ikResult.error, maxTorqueNm: 0,
                                    groundY: self.bridge.groundHeightY)
                return
            }

            // Smoothed IK output, dated at the center of the SG window.
            // This is what ID and muscle solves operate on — it matches dq/ddq
            // temporally, which is essential for correct inverse dynamics.
            var smoothedAngles: [String: Double] = [:]
            for i in 0..<min(numDOFs, dofNames.count) {
                smoothedAngles[dofNames[i]] = smoothedQ[i]
            }
            let smoothedIkOutput = IKOutput(
                jointAngles: smoothedAngles,
                error: ikResult.error,
                timestamp: centerTimestamp
            )

            // --- ID on SG-smoothed q, dq, ddq ---
            let smoothedQNS: [NSNumber] = smoothedQ.map { NSNumber(value: $0) }
            let smoothedDQNS: [NSNumber] = smoothedDQ.map { NSNumber(value: $0) }
            let smoothedDDQNS: [NSNumber] = smoothedDDQ.map { NSNumber(value: $0) }

            var idOutput: IDOutput?
            var idTime = 0.0
            var maxTorqueNm = 0.0

            let idStart = CACurrentMediaTime()
            // Use the GRF-aware ID solver. It runs Nimble's near-CoP
            // multi-contact inverse dynamics which auto-detects foot contact
            // and decomposes the system wrench into GRFs + joint torques.
            if let idResult = self.bridge.solveIDGRF(
                withJointAngles: smoothedQNS,
                jointVelocities: smoothedDQNS,
                jointAccelerations: smoothedDDQNS
            ) {
                idTime = (CACurrentMediaTime() - idStart) * 1000.0

                var torques: [String: Double] = [:]
                for i in 0..<min(idResult.jointTorques.count, dofNames.count) {
                    let t = idResult.jointTorques[i].doubleValue
                    torques[dofNames[i]] = t
                    if abs(t) > maxTorqueNm { maxTorqueNm = abs(t) }
                }

                var out = IDOutput(jointTorques: torques, timestamp: centerTimestamp)
                func toSimd(_ arr: [NSNumber]) -> SIMD3<Double> {
                    guard arr.count >= 3 else { return .zero }
                    return SIMD3<Double>(arr[0].doubleValue, arr[1].doubleValue, arr[2].doubleValue)
                }
                out.leftFootForce  = toSimd(idResult.leftFootForce)
                out.rightFootForce = toSimd(idResult.rightFootForce)
                out.leftFootCoP    = toSimd(idResult.leftFootCoP)
                out.rightFootCoP   = toSimd(idResult.rightFootCoP)
                out.leftFootInContact  = idResult.leftFootInContact
                out.rightFootInContact = idResult.rightFootInContact
                out.rootResidualNorm   = idResult.rootResidualNorm
                idOutput = out
            }

            // --- Muscle static optimization (on same SG-centered state) ---
            // Production path: real moment arms from FK + soft-equality QP +
            // real Hill-model force-length/velocity. Requires that the
            // skeleton is at the smoothed pose, which solveIDGRF() already
            // sets when it runs — MomentArmComputer picks up from there.
            var muscleOutput: MuscleOutput?
            var muscleTime = 0.0

            if let id = idOutput {
                // IMPORTANT: jointTorques arrives as a dictionary, which has
                // no defined order. We must force a consistent ordering here
                // so that `dofNames[i]` refers to the same DOF as `torques[i]`
                // AND as the j-th column of the moment-arm matrix. Use the
                // DOF names from IK (they are in skeleton order).
                let torqueKeys: [String] = ikResult.dofNames as [String]
                let torqueVals = torqueKeys.map { NSNumber(value: id.jointTorques[$0] ?? 0) }

                // Compute the moment-arm matrix R(q) at the smoothed pose.
                // MomentArmComputer runs its own FK so it expects the pose
                // to be driven through setPositions. Pass the SG-smoothed q.
                let momentArms = self.momentArmComputer.computeMomentArms(
                    withJointAngles: smoothedQNS,
                    dofNames: torqueKeys
                ) ?? []
                let muscleLengthsNS = self.momentArmComputer.currentMuscleLengths
                let muscleNamesNS = self.momentArmComputer.muscleNames
                let maxForcesNS = self.momentArmComputer.maxIsometricForces
                let optimalFibersNS = self.momentArmComputer.optimalFiberLengths
                let tendonSlacksNS = self.momentArmComputer.tendonSlackLengths
                let pennationsNS = self.momentArmComputer.pennationAngles

                if !momentArms.isEmpty && muscleNamesNS.count > 0 {
                    // dt for fiber-velocity finite differencing. We use the
                    // SG-window centered timestamp vs. the previous frame's.
                    let nowSec = centerTimestamp
                    let dt = max(nowSec - (self.lastMuscleSolveTimestamp ?? nowSec - 0.0167),
                                 1e-3)
                    self.lastMuscleSolveTimestamp = nowSec

                    if let result = self.muscleSolver.solveReal(
                        withJointTorques: torqueVals,
                        momentArms: momentArms,
                        muscleNames: muscleNamesNS,
                        muscleLengths: muscleLengthsNS,
                        maxForces: maxForcesNS,
                        optimalFiberLengths: optimalFibersNS,
                        tendonSlackLengths: tendonSlacksNS,
                        pennationAngles: pennationsNS,
                        jointVelocities: smoothedDQNS,
                        dofNames: torqueKeys,
                        dt: dt,
                        softPenalty: 1.0
                    ) {
                        muscleTime = result.solveTimeMs

                        var activations: [String: Double] = [:]
                        var forces: [String: Double] = [:]
                        for i in 0..<result.muscleNames.count {
                            activations[result.muscleNames[i]] = result.activations[i].doubleValue
                            forces[result.muscleNames[i]] = result.forces[i].doubleValue
                        }
                        muscleOutput = MuscleOutput(
                            activations: activations,
                            forces: forces,
                            converged: result.converged,
                            timestamp: centerTimestamp
                        )
                    }
                }
            }

            // Record if enabled (history always uses the SG-centered timestamp
            // so downstream .mot/.sto exports are temporally consistent).
            if self.isRecordingResults {
                self.ikHistory.append((centerTimestamp, smoothedAngles, ikResult.error))
                if let id = idOutput {
                    self.idHistory.append((centerTimestamp, id.jointTorques))
                }
            }

            self.publishResults(ik: smoothedIkOutput, id: idOutput, muscle: muscleOutput,
                                ikTime: ikTime, idTime: idTime, muscleTime: muscleTime,
                                ikResidual: ikResult.error, maxTorqueNm: maxTorqueNm,
                                groundY: self.bridge.groundHeightY)
        }
    }

    private func publishResults(ik: IKOutput, id: IDOutput?, muscle: MuscleOutput?,
                                ikTime: Double, idTime: Double, muscleTime: Double,
                                ikResidual: Double, maxTorqueNm: Double,
                                groundY: Double) {
        let mass = max(totalMassKg, 1e-6)
        let torquePerKg = maxTorqueNm / mass
        // Vertical load on each foot as a fraction of body weight. Useful for
        // validating stance vs. swing phase and impact peaks during gait.
        let weightN = mass * 9.81
        let leftLoad  = (id?.leftFootForce.y  ?? 0) / weightN
        let rightLoad = (id?.rightFootForce.y ?? 0) / weightN
        let rootResPerKg = (id?.rootResidualNorm ?? 0) / mass
        DispatchQueue.main.async { [weak self] in
            guard let self else { return }
            self.lastIKResult = ik
            self.lastIDResult = id
            self.lastMuscleResult = muscle
            self.ikSolveTimeMs = ikTime
            self.idSolveTimeMs = idTime
            self.muscleSolveTimeMs = muscleTime
            self.ikMarkerResidualMeters = ikResidual
            self.maxTorquePerKg = torquePerKg
            self.leftFootLoadFraction = leftLoad
            self.rightFootLoadFraction = rightLoad
            self.rootResidualPerKg = rootResPerKg
            self.groundHeightY = groundY
        }
    }

    // MARK: - Recording

    func startRecordingResults() {
        ikHistory.removeAll()
        idHistory.removeAll()
        isRecordingResults = true
    }

    func stopRecordingResults() {
        isRecordingResults = false
    }

    /// Export IK results as .mot file.
    func exportMOT(filename: String = "BioMotion_ik") throws -> URL {
        guard !ikHistory.isEmpty else { throw ExportError.noData }

        let startTime = ikHistory.first!.timestamp
        let allDOFs = Array(ikHistory.first!.angles.keys).sorted()

        var lines: [String] = []
        lines.append(filename)
        lines.append("version=1")
        lines.append("nRows=\(ikHistory.count)")
        lines.append("nColumns=\(allDOFs.count + 1)")
        lines.append("inDegrees=yes")
        lines.append("endheader")

        // Column headers
        lines.append("time\t" + allDOFs.joined(separator: "\t"))

        // Data rows
        for entry in ikHistory {
            let time = entry.timestamp - startTime
            var row = String(format: "%.6f", time)
            for dof in allDOFs {
                let angleRad = entry.angles[dof] ?? 0.0
                let angleDeg = angleRad * 180.0 / .pi
                row += String(format: "\t%.4f", angleDeg)
            }
            lines.append(row)
        }

        let content = lines.joined(separator: "\n")
        let url = FileManager.default.temporaryDirectory.appendingPathComponent("\(filename).mot")
        try content.write(to: url, atomically: true, encoding: .utf8)
        return url
    }

    /// Export ID results as .sto file.
    func exportSTO(filename: String = "BioMotion_id") throws -> URL {
        guard !idHistory.isEmpty else { throw ExportError.noData }

        let startTime = idHistory.first!.timestamp
        let allDOFs = Array(idHistory.first!.jointTorques.keys).sorted()

        var lines: [String] = []
        lines.append(filename)
        lines.append("version=1")
        lines.append("nRows=\(idHistory.count)")
        lines.append("nColumns=\(allDOFs.count + 1)")
        lines.append("inDegrees=no")
        lines.append("endheader")

        lines.append("time\t" + allDOFs.joined(separator: "\t"))

        for entry in idHistory {
            let time = entry.timestamp - startTime
            var row = String(format: "%.6f", time)
            for dof in allDOFs {
                row += String(format: "\t%.4f", entry.jointTorques[dof] ?? 0.0)
            }
            lines.append(row)
        }

        let content = lines.joined(separator: "\n")
        let url = FileManager.default.temporaryDirectory.appendingPathComponent("\(filename).sto")
        try content.write(to: url, atomically: true, encoding: .utf8)
        return url
    }

    enum ExportError: Error {
        case noData
    }
}
