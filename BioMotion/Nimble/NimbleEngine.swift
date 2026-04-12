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

    // State for finite-difference velocity/acceleration estimation
    private var prevAngles: [NSNumber]?
    private var prevVelocities: [NSNumber]?
    private var prevTimestamp: TimeInterval?

    // IK history for recording
    private(set) var ikHistory: [(timestamp: TimeInterval, angles: [String: Double], error: Double)] = []
    private(set) var idHistory: [(timestamp: TimeInterval, jointTorques: [String: Double])] = []
    private var isRecordingResults = false

    /// Load the bundled .osim model.
    func loadBundledModel() {
        guard let path = Bundle.main.path(forResource: "Rajagopal2016", ofType: "osim") else {
            print("NimbleEngine: Rajagopal2016.osim not found in bundle")
            return
        }
        solverQueue.async { [weak self] in
            guard let self else { return }
            let success = self.bridge.loadModel(fromPath: path)
            if success {
                self.muscleSolver.loadMuscles(fromOsimPath: path)
                self.momentArmComputer.parseMusclePaths(fromOsimPath: path)
            }
            DispatchQueue.main.async {
                self.isModelLoaded = success
                if success {
                    let modelMarkers = self.bridge.markerNames as [String]
                    print("NimbleEngine: Model loaded — \(self.bridge.numDOFs) DOFs, \(modelMarkers.count) markers, \(self.muscleSolver.numMuscles) muscles")
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

            // --- IK ---
            let ikStart = CACurrentMediaTime()
            guard let ikResult = self.bridge.solveIK(
                withMarkerPositions: positions,
                markerNames: names
            ) else { return }
            let ikTime = (CACurrentMediaTime() - ikStart) * 1000.0

            // Build named angles dictionary
            var angles: [String: Double] = [:]
            let dofNames = ikResult.dofNames
            for i in 0..<min(ikResult.jointAngles.count, dofNames.count) {
                angles[dofNames[i]] = ikResult.jointAngles[i].doubleValue
            }

            let ikOutput = IKOutput(
                jointAngles: angles,
                error: ikResult.error,
                timestamp: frame.timestamp
            )

            // --- ID (if we have previous frame data for finite differences) ---
            var idOutput: IDOutput?
            var idTime = 0.0

            if let prevAngles = self.prevAngles,
               let prevVelocities = self.prevVelocities,
               let prevTimestamp = self.prevTimestamp {

                let dt = frame.timestamp - prevTimestamp
                guard dt > 0.001 else {
                    self.prevAngles = ikResult.jointAngles
                    self.prevTimestamp = frame.timestamp
                    self.publishResults(ik: ikOutput, id: nil, muscle: nil,
                                        ikTime: ikTime, idTime: 0, muscleTime: 0)
                    return
                }

                // Compute velocities (finite difference)
                let numDOFs = ikResult.jointAngles.count
                var velocities: [NSNumber] = []
                var accelerations: [NSNumber] = []

                for i in 0..<numDOFs {
                    let q = ikResult.jointAngles[i].doubleValue
                    let q_prev = prevAngles[i].doubleValue
                    let v = (q - q_prev) / dt
                    velocities.append(NSNumber(value: v))

                    let v_prev = prevVelocities[i].doubleValue
                    let a = (v - v_prev) / dt
                    accelerations.append(NSNumber(value: a))
                }

                // Run ID
                let idStart = CACurrentMediaTime()
                if let idResult = self.bridge.solveID(
                    withJointAngles: ikResult.jointAngles,
                    jointVelocities: velocities,
                    jointAccelerations: accelerations
                ) {
                    idTime = (CACurrentMediaTime() - idStart) * 1000.0

                    var torques: [String: Double] = [:]
                    for i in 0..<min(idResult.jointTorques.count, dofNames.count) {
                        torques[dofNames[i]] = idResult.jointTorques[i].doubleValue
                    }
                    idOutput = IDOutput(jointTorques: torques, timestamp: frame.timestamp)
                }

                self.prevVelocities = velocities
            } else {
                // First frame — no velocities yet
                self.prevVelocities = Array(repeating: NSNumber(value: 0.0),
                                            count: ikResult.jointAngles.count)
            }

            self.prevAngles = ikResult.jointAngles
            self.prevTimestamp = frame.timestamp

            // --- Muscle static optimization (after ID) ---
            var muscleOutput: MuscleOutput?
            var muscleTime = 0.0

            if let id = idOutput {
                let muscleStart = CACurrentMediaTime()
                if let result = self.muscleSolver.solve(
                    withJointTorques: Array(id.jointTorques.values).map { NSNumber(value: $0) },
                    jointAngles: ikResult.jointAngles,
                    jointVelocities: self.prevVelocities ?? [],
                    dofNames: Array(id.jointTorques.keys)
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
                        timestamp: frame.timestamp
                    )
                }
            }

            // Record if enabled
            if self.isRecordingResults {
                self.ikHistory.append((frame.timestamp, angles, ikResult.error))
                if let id = idOutput {
                    self.idHistory.append((frame.timestamp, id.jointTorques))
                }
            }

            self.publishResults(ik: ikOutput, id: idOutput, muscle: muscleOutput,
                              ikTime: ikTime, idTime: idTime, muscleTime: muscleTime)
        }
    }

    private func publishResults(ik: IKOutput, id: IDOutput?, muscle: MuscleOutput?,
                                ikTime: Double, idTime: Double, muscleTime: Double) {
        DispatchQueue.main.async { [weak self] in
            self?.lastIKResult = ik
            self?.lastIDResult = id
            self?.lastMuscleResult = muscle
            self?.ikSolveTimeMs = ikTime
            self?.idSolveTimeMs = idTime
            self?.muscleSolveTimeMs = muscleTime
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
