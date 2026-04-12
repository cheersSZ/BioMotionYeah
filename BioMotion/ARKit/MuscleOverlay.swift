import UIKit
import RealityKit
import simd

/// Renders 3D muscle visualization as colored capsules positioned anatomically,
/// overlaid on the camera feed via RealityKit.
final class MuscleOverlay {

    /// Definition of a visual muscle representation.
    struct MuscleDef {
        let name: String           // Matches MuscleSolver muscle name
        let startJoint: String     // ARKit joint name for origin
        let endJoint: String       // ARKit joint name for insertion
        let offsetStart: SIMD3<Float>  // Offset from start joint (meters)
        let offsetEnd: SIMD3<Float>    // Offset from end joint (meters)
        let radius: Float          // Visual thickness (meters)
        let side: Side

        enum Side { case left, right, center }
    }

    private var muscleEntities: [String: ModelEntity] = [:]
    private var anchor: AnchorEntity?

    // Pre-defined muscle visual positions (approximate anatomical placement)
    static let muscleDefs: [MuscleDef] = {
        var defs: [MuscleDef] = []

        // Helper to create bilateral pair
        func bilateral(_ name: String, startJoint: String, endJoint: String,
                       offsetStart: SIMD3<Float> = .zero, offsetEnd: SIMD3<Float> = .zero,
                       radius: Float = 0.018) {
            let rStart = startJoint.replacingOccurrences(of: "left", with: "right")
            let rEnd = endJoint.replacingOccurrences(of: "left", with: "right")
            defs.append(MuscleDef(name: name + "_r", startJoint: rStart, endJoint: rEnd,
                                  offsetStart: SIMD3(offsetStart.x, offsetStart.y, offsetStart.z),
                                  offsetEnd: SIMD3(offsetEnd.x, offsetEnd.y, offsetEnd.z),
                                  radius: radius, side: .right))
            defs.append(MuscleDef(name: name + "_l", startJoint: startJoint, endJoint: endJoint,
                                  offsetStart: SIMD3(-offsetStart.x, offsetStart.y, offsetStart.z),
                                  offsetEnd: SIMD3(-offsetEnd.x, offsetEnd.y, offsetEnd.z),
                                  radius: radius, side: .left))
        }

        // === Thigh — Anterior ===

        // Rectus femoris (hip → knee, front of thigh)
        bilateral("recfem",
                  startJoint: "left_upLeg_joint", endJoint: "left_leg_joint",
                  offsetStart: SIMD3(0, 0, 0.04), offsetEnd: SIMD3(0, 0.03, 0.03),
                  radius: 0.022)

        // Vastus medialis (mid-thigh → knee, inner front)
        bilateral("vasmed",
                  startJoint: "left_upLeg_joint", endJoint: "left_leg_joint",
                  offsetStart: SIMD3(-0.02, -0.08, 0.02), offsetEnd: SIMD3(-0.01, 0.02, 0.02),
                  radius: 0.02)

        // Vastus lateralis (mid-thigh → knee, outer front)
        bilateral("vaslat",
                  startJoint: "left_upLeg_joint", endJoint: "left_leg_joint",
                  offsetStart: SIMD3(0.03, -0.05, 0.02), offsetEnd: SIMD3(0.02, 0.02, 0.02),
                  radius: 0.02)

        // === Thigh — Posterior ===

        // Semimembranosus (hip → knee, back of thigh)
        bilateral("semimem",
                  startJoint: "left_upLeg_joint", endJoint: "left_leg_joint",
                  offsetStart: SIMD3(-0.01, 0, -0.04), offsetEnd: SIMD3(-0.02, 0.02, -0.02),
                  radius: 0.018)

        // Biceps femoris long head
        bilateral("bflh",
                  startJoint: "left_upLeg_joint", endJoint: "left_leg_joint",
                  offsetStart: SIMD3(0.02, 0, -0.04), offsetEnd: SIMD3(0.03, 0.02, -0.01),
                  radius: 0.016)

        // === Lower Leg ===

        // Gastrocnemius medial (knee → ankle, back of calf)
        bilateral("gasmed",
                  startJoint: "left_leg_joint", endJoint: "left_foot_joint",
                  offsetStart: SIMD3(-0.01, -0.02, -0.03), offsetEnd: SIMD3(0, 0.02, -0.01),
                  radius: 0.02)

        // Gastrocnemius lateral
        bilateral("gaslat",
                  startJoint: "left_leg_joint", endJoint: "left_foot_joint",
                  offsetStart: SIMD3(0.02, -0.02, -0.03), offsetEnd: SIMD3(0, 0.02, -0.01),
                  radius: 0.018)

        // Soleus (below knee → ankle, deep calf)
        bilateral("soleus",
                  startJoint: "left_leg_joint", endJoint: "left_foot_joint",
                  offsetStart: SIMD3(0, -0.06, -0.02), offsetEnd: SIMD3(0, 0.02, -0.01),
                  radius: 0.022)

        // Tibialis anterior (knee → foot, front of shin)
        bilateral("tibant",
                  startJoint: "left_leg_joint", endJoint: "left_foot_joint",
                  offsetStart: SIMD3(0.01, -0.03, 0.03), offsetEnd: SIMD3(0.01, 0, 0.02),
                  radius: 0.014)

        // === Hip / Gluteal ===

        // Gluteus maximus (3 parts in the model — use glmax1 as representative)
        bilateral("glmax1",
                  startJoint: "hips_joint", endJoint: "left_upLeg_joint",
                  offsetStart: SIMD3(0.06, -0.02, -0.06), offsetEnd: SIMD3(0.02, -0.05, -0.03),
                  radius: 0.03)

        // Gluteus medius
        bilateral("glmed1",
                  startJoint: "hips_joint", endJoint: "left_upLeg_joint",
                  offsetStart: SIMD3(0.08, 0.02, -0.02), offsetEnd: SIMD3(0.04, -0.02, 0),
                  radius: 0.022)

        // Psoas (lumbar → hip, deep hip flexor)
        bilateral("psoas",
                  startJoint: "spine_1_joint", endJoint: "left_upLeg_joint",
                  offsetStart: SIMD3(0.02, -0.03, 0.03), offsetEnd: SIMD3(0, 0.02, 0.01),
                  radius: 0.014)

        // === Trunk ===

        // Erector spinae (approximate as center muscles)
        defs.append(MuscleDef(name: "ercspn_r",
                              startJoint: "hips_joint", endJoint: "spine_4_joint",
                              offsetStart: SIMD3(0.03, 0, -0.06),
                              offsetEnd: SIMD3(0.03, 0, -0.04),
                              radius: 0.02, side: .right))
        defs.append(MuscleDef(name: "ercspn_l",
                              startJoint: "hips_joint", endJoint: "spine_4_joint",
                              offsetStart: SIMD3(-0.03, 0, -0.06),
                              offsetEnd: SIMD3(-0.03, 0, -0.04),
                              radius: 0.02, side: .left))

        return defs
    }()

    func setup(anchor: AnchorEntity) {
        self.anchor = anchor
    }

    /// Update muscle visualization with current joint positions and activations.
    func update(joints: [TrackedJoint], activations: [String: Double]) {
        guard let anchor else { return }

        // Build joint position lookup
        var jointPositions: [String: SIMD3<Float>] = [:]
        for joint in joints where joint.isTracked {
            jointPositions[joint.id] = joint.worldPosition
        }

        for def in Self.muscleDefs {
            guard let startPos = jointPositions[def.startJoint],
                  let endPos = jointPositions[def.endJoint] else {
                muscleEntities[def.name]?.isEnabled = false
                continue
            }

            let worldStart = startPos + def.offsetStart
            let worldEnd = endPos + def.offsetEnd
            let midpoint = (worldStart + worldEnd) / 2.0
            let length = simd_length(worldEnd - worldStart)

            guard length > 0.01 else {
                muscleEntities[def.name]?.isEnabled = false
                continue
            }

            let activation = activations[def.name] ?? 0.01
            let color = activationColor(activation)

            let entity: ModelEntity
            if let existing = muscleEntities[def.name] {
                entity = existing
                // Update mesh for new length
                entity.model?.mesh = MeshResource.generateBox(
                    size: SIMD3(def.radius * 2, def.radius * 2, length),
                    cornerRadius: def.radius
                )
                // Update material color
                entity.model?.materials = [SimpleMaterial(color: color, isMetallic: false)]
                entity.isEnabled = true
            } else {
                let mesh = MeshResource.generateBox(
                    size: SIMD3(def.radius * 2, def.radius * 2, length),
                    cornerRadius: def.radius
                )
                let material = SimpleMaterial(color: color, isMetallic: false)
                entity = ModelEntity(mesh: mesh, materials: [material])
                anchor.addChild(entity)
                muscleEntities[def.name] = entity
            }

            entity.position = midpoint
            entity.look(at: worldEnd, from: midpoint, relativeTo: nil)
        }
    }

    /// Remove all muscle entities.
    func clear() {
        for (_, entity) in muscleEntities {
            entity.removeFromParent()
        }
        muscleEntities.removeAll()
    }

    // MARK: - Color Mapping

    private func activationColor(_ activation: Double) -> UIColor {
        let a = Float(max(0, min(1, activation)))

        // Blue (rest) → Cyan → Green → Yellow → Red (max)
        let r: Float
        let g: Float
        let b: Float

        if a < 0.25 {
            // Blue → Cyan
            let t = a / 0.25
            r = 0; g = t; b = 1.0
        } else if a < 0.5 {
            // Cyan → Green
            let t = (a - 0.25) / 0.25
            r = 0; g = 1.0; b = 1.0 - t
        } else if a < 0.75 {
            // Green → Yellow
            let t = (a - 0.5) / 0.25
            r = t; g = 1.0; b = 0
        } else {
            // Yellow → Red
            let t = (a - 0.75) / 0.25
            r = 1.0; g = 1.0 - t; b = 0
        }

        return UIColor(red: CGFloat(r), green: CGFloat(g), blue: CGFloat(b), alpha: 0.6)
    }
}
