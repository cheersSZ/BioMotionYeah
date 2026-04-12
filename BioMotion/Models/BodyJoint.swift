import simd
import ARKit

/// A single tracked joint with its 3D position in world space.
struct TrackedJoint: Identifiable {
    let id: String          // ARSkeleton.JointName rawValue
    let name: String        // Human-readable name
    let worldPosition: SIMD3<Float>  // Meters, world space (Y-up)
    let isTracked: Bool
}

/// A complete body frame — all joints at one instant.
struct BodyFrame {
    let timestamp: TimeInterval
    let frameNumber: Int
    let joints: [TrackedJoint]

    /// Root (hip) position in world space.
    var rootPosition: SIMD3<Float>? {
        joints.first(where: { $0.id == "hips_joint" })?.worldPosition
    }
}

/// Maps ARKit skeleton joint names to OpenSim marker names (Rajagopal2016).
enum JointMapping {
    struct Mapping {
        let arkitName: String
        let opensimName: String
        let displayName: String
    }

    /// Primary mappings: ARKit joints that correspond directly to OpenSim joint centers.
    static let primary: [Mapping] = [
        // Pelvis / Root
        Mapping(arkitName: "hips_joint", opensimName: "PELVIS", displayName: "Pelvis"),
        // Lower body
        Mapping(arkitName: "left_upLeg_joint", opensimName: "LHJC", displayName: "L Hip"),
        Mapping(arkitName: "right_upLeg_joint", opensimName: "RHJC", displayName: "R Hip"),
        Mapping(arkitName: "left_leg_joint", opensimName: "LKJC", displayName: "L Knee"),
        Mapping(arkitName: "right_leg_joint", opensimName: "RKJC", displayName: "R Knee"),
        Mapping(arkitName: "left_foot_joint", opensimName: "LAJC", displayName: "L Ankle"),
        Mapping(arkitName: "right_foot_joint", opensimName: "RAJC", displayName: "R Ankle"),
        Mapping(arkitName: "left_toes_joint", opensimName: "LTOE", displayName: "L Toe"),
        Mapping(arkitName: "right_toes_joint", opensimName: "RTOE", displayName: "R Toe"),
        // Spine
        Mapping(arkitName: "spine_1_joint", opensimName: "SPINE_L", displayName: "Lower Spine"),
        Mapping(arkitName: "spine_4_joint", opensimName: "SPINE_M", displayName: "Mid Spine"),
        Mapping(arkitName: "spine_7_joint", opensimName: "C7", displayName: "C7"),
        Mapping(arkitName: "neck_1_joint", opensimName: "NECK", displayName: "Neck"),
        Mapping(arkitName: "head_joint", opensimName: "HEAD", displayName: "Head"),
        // Upper body
        Mapping(arkitName: "left_shoulder_1_joint", opensimName: "LSJC", displayName: "L Shoulder"),
        Mapping(arkitName: "right_shoulder_1_joint", opensimName: "RSJC", displayName: "R Shoulder"),
        Mapping(arkitName: "left_forearm_joint", opensimName: "LEJC", displayName: "L Elbow"),
        Mapping(arkitName: "right_forearm_joint", opensimName: "REJC", displayName: "R Elbow"),
        Mapping(arkitName: "left_hand_joint", opensimName: "LWJC", displayName: "L Wrist"),
        Mapping(arkitName: "right_hand_joint", opensimName: "RWJC", displayName: "R Wrist"),
    ]

    /// Bones: pairs of joint indices (into `primary`) to draw as skeleton lines.
    static let bones: [(Int, Int)] = [
        // Spine chain
        (0, 9), (9, 10), (10, 11), (11, 12), (12, 13),
        // Left leg
        (0, 1), (1, 3), (3, 5), (5, 7),
        // Right leg
        (0, 2), (2, 4), (4, 6), (6, 8),
        // Left arm
        (11, 14), (14, 16), (16, 18),
        // Right arm
        (11, 15), (15, 17), (17, 19),
    ]
}
