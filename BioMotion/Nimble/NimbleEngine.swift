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

    /// Selects which inverse-dynamics path runs in `runDynamicsAndMuscles`.
    ///
    /// - `withGRF` (default): the production path. Uses
    ///   `bridge.solveIDGRF`, which auto-estimates ground reaction forces
    ///   and decomposes the system wrench into per-foot contacts plus joint
    ///   torques. Required for any motion with ground contact.
    /// - `noGRF`: diagnostic path. Calls `bridge.solveID`, which assumes
    ///   zero external forces. Joint torques become whatever is needed to
    ///   reproduce the kinematics from gravity + inertia alone, so any
    ///   left/right asymmetry observed under this mode cannot have come
    ///   from the GRF estimator. Useful for isolating bilateral muscle
    ///   activation bias to either the GRF path (asymmetric in `.withGRF`,
    ///   symmetric in `.noGRF`) or the rest of the pipeline.
    enum IDMode {
        case withGRF
        case noGRF
    }

    /// Active inverse-dynamics mode. Read on each frame. Defaults to
    /// `.withGRF` so the live ARKit pipeline behaves exactly as before.
    /// Toggled from the offline UI for diagnostic A/B runs.
    var idMode: IDMode = .withGRF

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
    /// Per-frame snapshot captured during a recording session; populated alongside
    /// `ikHistory`/`idHistory` whenever the SG filter has warmed up.
    /// Holds GRF + root-residual data the slim `idHistory` tuple drops, plus
    /// muscle activations / forces / convergence.
    struct DynamicsHistoryEntry {
        let timestamp: TimeInterval
        let ikError: Double
        let leftFootForce: SIMD3<Double>
        let rightFootForce: SIMD3<Double>
        let leftFootInContact: Bool
        let rightFootInContact: Bool
        let rootResidualNorm: Double
        let maxAbsTorqueNm: Double
        let muscle: MuscleOutput?
    }
    private(set) var dynamicsHistory: [DynamicsHistoryEntry] = []
    private var isRecordingResults = false

    /// Block the caller until every queued solver job has finished. Useful for
    /// batch / offline processing where we dispatch a sequence of frames and
    /// need to know all results have been published before reading history.
    func waitForPendingSolverWork() {
        solverQueue.sync {}
    }

    /// Load a specific .osim model from disk.
    func loadModel(fromPath path: String, completion: ((Bool) -> Void)? = nil) {
        solverQueue.async { [weak self] in
            guard let self else { return }
            let success = self.bridge.loadModel(fromPath: path)
            if success {
                self.muscleSolver.loadMuscles(fromOsimPath: path)
                self.momentArmComputer.parseMusclePaths(fromOsimPath: path,
                                                        from: self.bridge)
                self.resetAnalysisState()
            }
            DispatchQueue.main.async {
                self.isModelLoaded = success
                if success {
                    self.totalMassKg = self.bridge.totalMass
                    let modelMarkers = self.bridge.markerNames as [String]
                    print("NimbleEngine: Model loaded — \(self.bridge.numDOFs) DOFs, \(modelMarkers.count) markers, \(self.muscleSolver.numMuscles) muscles, mass \(String(format: "%.1f", self.totalMassKg)) kg")
                }
                completion?(success)
            }
        }
    }

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
        loadModel(fromPath: path)
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
            self.processRawKinematics(
                rawAngles: ikResult.jointAngles.map(\.doubleValue),
                dofNames: ikResult.dofNames as [String],
                timestamp: frame.timestamp,
                ikError: ikResult.error,
                ikTime: ikTime
            )
        }
    }

    func prepareImportedMotion(_ series: ImportedMotionSeries) -> ImportedMotionPreparation? {
        guard isModelLoaded else { return nil }
        return ImportedMotionPreparer.prepare(
            series: series,
            modelDOFNames: bridge.dofNames as [String]
        )
    }

    func processImportedMotionFrame(_ frame: PreparedMotionFrame) {
        guard isModelLoaded else { return }
        solverQueue.async { [weak self] in
            self?.processRawKinematics(
                rawAngles: frame.positions,
                dofNames: frame.modelDOFNames,
                timestamp: frame.timestamp,
                ikError: 0,
                ikTime: 0
            )
        }
    }

    func resetMotionFilters() {
        solverQueue.async { [weak self] in
            self?.resetAnalysisState()
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
        dynamicsHistory.removeAll()
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

    private func processRawKinematics(
        rawAngles: [Double],
        dofNames: [String],
        timestamp: TimeInterval,
        ikError: Double,
        ikTime: Double
    ) {
        let numDOFs = rawAngles.count
        ensureDofFilters(count: numDOFs)

        var smoothedQ = [Double](); smoothedQ.reserveCapacity(numDOFs)
        var smoothedDQ = [Double](); smoothedDQ.reserveCapacity(numDOFs)
        var smoothedDDQ = [Double](); smoothedDDQ.reserveCapacity(numDOFs)
        var centerTimestamp = timestamp
        var sgWarmedUp = true

        // Push the new sample into EVERY DOF filter regardless of warm-up state.
        // The previous early-`break` only pushed DOF 0 during warm-up, so DOFs
        // 1..N stayed empty until DOF 0 finished its 9-sample window — multiplying
        // effective warm-up to 9*N frames and silently breaking offline batch
        // on strided input where the budget is just ~100 frames.
        for i in 0..<numDOFs {
            if let out = dofFilters[i].push(rawAngles[i], timestamp: timestamp) {
                smoothedQ.append(out.pos)
                smoothedDQ.append(out.vel)
                smoothedDDQ.append(out.acc)
                centerTimestamp = out.center
            } else {
                sgWarmedUp = false
            }
        }

        var liveAngles: [String: Double] = [:]
        for i in 0..<min(numDOFs, dofNames.count) {
            liveAngles[dofNames[i]] = rawAngles[i]
        }
        let liveIkOutput = IKOutput(
            jointAngles: liveAngles,
            error: ikError,
            timestamp: timestamp
        )

        guard sgWarmedUp else {
            let warmupGroundY = (idMode == .noGRF) ? 0.0 : bridge.groundHeightY
            publishResults(ik: liveIkOutput, id: nil, muscle: nil,
                           ikTime: ikTime, idTime: 0, muscleTime: 0,
                           ikResidual: ikError, maxTorqueNm: 0,
                           groundY: warmupGroundY)
            return
        }

        var smoothedAngles: [String: Double] = [:]
        for i in 0..<min(numDOFs, dofNames.count) {
            smoothedAngles[dofNames[i]] = smoothedQ[i]
        }
        let smoothedIkOutput = IKOutput(
            jointAngles: smoothedAngles,
            error: ikError,
            timestamp: centerTimestamp
        )

        let dynamics = runDynamicsAndMuscles(
            smoothedQ: smoothedQ,
            smoothedDQ: smoothedDQ,
            smoothedDDQ: smoothedDDQ,
            dofNames: dofNames,
            centerTimestamp: centerTimestamp
        )

        if isRecordingResults {
            ikHistory.append((centerTimestamp, smoothedAngles, ikError))
            if let id = dynamics.idOutput {
                idHistory.append((centerTimestamp, id.jointTorques))
            }
            dynamicsHistory.append(
                DynamicsHistoryEntry(
                    timestamp: centerTimestamp,
                    ikError: ikError,
                    leftFootForce: dynamics.idOutput?.leftFootForce ?? .zero,
                    rightFootForce: dynamics.idOutput?.rightFootForce ?? .zero,
                    leftFootInContact: dynamics.idOutput?.leftFootInContact ?? false,
                    rightFootInContact: dynamics.idOutput?.rightFootInContact ?? false,
                    rootResidualNorm: dynamics.idOutput?.rootResidualNorm ?? 0,
                    maxAbsTorqueNm: dynamics.maxTorqueNm,
                    muscle: dynamics.muscleOutput
                )
            )
        }

        // In noGRF mode the bridge's ground bars are not updated this frame,
        // so reading `bridge.groundHeightY` would surface stale state from a
        // prior `.withGRF` run. Publish 0 instead — the GRF/CoP/load-fraction
        // fields above are already zeroed for the same reason.
        let publishedGroundY = (idMode == .noGRF) ? 0.0 : bridge.groundHeightY
        publishResults(
            ik: smoothedIkOutput,
            id: dynamics.idOutput,
            muscle: dynamics.muscleOutput,
            ikTime: ikTime,
            idTime: dynamics.idTime,
            muscleTime: dynamics.muscleTime,
            ikResidual: ikError,
            maxTorqueNm: dynamics.maxTorqueNm,
            groundY: publishedGroundY
        )
    }

    private func runDynamicsAndMuscles(
        smoothedQ: [Double],
        smoothedDQ: [Double],
        smoothedDDQ: [Double],
        dofNames: [String],
        centerTimestamp: TimeInterval
    ) -> (idOutput: IDOutput?, muscleOutput: MuscleOutput?, idTime: Double, muscleTime: Double, maxTorqueNm: Double) {
        let smoothedQNS = smoothedQ.map { NSNumber(value: $0) }
        let smoothedDQNS = smoothedDQ.map { NSNumber(value: $0) }
        let smoothedDDQNS = smoothedDDQ.map { NSNumber(value: $0) }

        var idOutput: IDOutput?
        var idTime = 0.0
        var maxTorqueNm = 0.0

        let idStart = CACurrentMediaTime()

        // Select the ID variant up front so the no-GRF diagnostic path stays
        // structurally clean: the resulting `IDOutput` carries only torques,
        // with all GRF/CoP/contact/residual fields explicitly zeroed (no
        // accidental bleed-through from a previous .withGRF frame's state).
        let idResult: NimbleIDResult?
        let isGRFPath: Bool
        switch idMode {
        case .withGRF:
            idResult = bridge.solveIDGRF(
                withJointAngles: smoothedQNS,
                jointVelocities: smoothedDQNS,
                jointAccelerations: smoothedDDQNS
            )
            isGRFPath = true
        case .noGRF:
            idResult = bridge.solveID(
                withJointAngles: smoothedQNS,
                jointVelocities: smoothedDQNS,
                jointAccelerations: smoothedDDQNS
            )
            isGRFPath = false
        }

        if let idResult {
            idTime = (CACurrentMediaTime() - idStart) * 1000.0

            var torques: [String: Double] = [:]
            for i in 0..<min(idResult.jointTorques.count, dofNames.count) {
                let t = idResult.jointTorques[i].doubleValue
                torques[dofNames[i]] = t
                if abs(t) > maxTorqueNm { maxTorqueNm = abs(t) }
            }

            var out = IDOutput(jointTorques: torques, timestamp: centerTimestamp)
            if isGRFPath {
                func toSimd(_ arr: [NSNumber]) -> SIMD3<Double> {
                    guard arr.count >= 3 else { return .zero }
                    return SIMD3<Double>(arr[0].doubleValue, arr[1].doubleValue, arr[2].doubleValue)
                }
                out.leftFootForce = toSimd(idResult.leftFootForce)
                out.rightFootForce = toSimd(idResult.rightFootForce)
                out.leftFootCoP = toSimd(idResult.leftFootCoP)
                out.rightFootCoP = toSimd(idResult.rightFootCoP)
                out.leftFootInContact = idResult.leftFootInContact
                out.rightFootInContact = idResult.rightFootInContact
                out.rootResidualNorm = idResult.rootResidualNorm
            }
            // .noGRF: leave all GRF fields at their default zeros set above.
            idOutput = out
        }

        var muscleOutput: MuscleOutput?
        var muscleTime = 0.0

        if let id = idOutput {
            let torqueVals = dofNames.map { NSNumber(value: id.jointTorques[$0] ?? 0) }
            let momentArms = momentArmComputer.computeMomentArms(
                withJointAngles: smoothedQNS,
                dofNames: dofNames
            ) ?? []
            let muscleLengthsNS = momentArmComputer.currentMuscleLengths
            let muscleNamesNS = momentArmComputer.muscleNames
            let maxForcesNS = momentArmComputer.maxIsometricForces
            let optimalFibersNS = momentArmComputer.optimalFiberLengths
            let tendonSlacksNS = momentArmComputer.tendonSlackLengths
            let pennationsNS = momentArmComputer.pennationAngles

            if !momentArms.isEmpty && muscleNamesNS.count > 0 {
                let nowSec = centerTimestamp
                let dt = max(nowSec - (lastMuscleSolveTimestamp ?? nowSec - 0.0167), 1e-3)
                lastMuscleSolveTimestamp = nowSec

                if let result = muscleSolver.solveReal(
                    withJointTorques: torqueVals,
                    momentArms: momentArms,
                    muscleNames: muscleNamesNS,
                    muscleLengths: muscleLengthsNS,
                    maxForces: maxForcesNS,
                    optimalFiberLengths: optimalFibersNS,
                    tendonSlackLengths: tendonSlacksNS,
                    pennationAngles: pennationsNS,
                    jointVelocities: smoothedDQNS,
                    dofNames: dofNames,
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

        return (idOutput, muscleOutput, idTime, muscleTime, maxTorqueNm)
    }

    private func ensureDofFilters(count: Int) {
        if dofFilters.count != count {
            dofFilters = (0..<count).map { _ in SavitzkyGolayFilter() }
        }
    }

    private func resetAnalysisState() {
        dofFilters.removeAll(keepingCapacity: false)
        lastMuscleSolveTimestamp = nil
        ikHistory.removeAll(keepingCapacity: false)
        idHistory.removeAll(keepingCapacity: false)
        dynamicsHistory.removeAll(keepingCapacity: false)
        DispatchQueue.main.async { [weak self] in
            guard let self else { return }
            self.lastIKResult = nil
            self.lastIDResult = nil
            self.lastMuscleResult = nil
            self.ikSolveTimeMs = 0
            self.idSolveTimeMs = 0
            self.muscleSolveTimeMs = 0
            self.ikMarkerResidualMeters = 0
            self.maxTorquePerKg = 0
            self.leftFootLoadFraction = 0
            self.rightFootLoadFraction = 0
            self.rootResidualPerKg = 0
            self.groundHeightY = self.bridge.groundHeightY
        }
    }
}
