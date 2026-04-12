#import "MuscleSolver.h"
#import <QuartzCore/QuartzCore.h>

#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <algorithm>

#include <tinyxml2.h>
#include "osqp.h"


// MARK: - Millard2012 Force-Length-Velocity Model

namespace muscle {

struct MuscleParams {
    std::string name;
    double maxIsometricForce;      // F0, Newtons
    double optimalFiberLength;     // l0, meters
    double tendonSlackLength;      // lTs, meters
    double pennationAngleAtOptimal; // alpha0, radians
    double maxContractionVelocity; // vmax, l0/s (default 10)

    // Which DOFs this muscle spans (for moment arm computation)
    // Simplified: map from DOF name to approximate moment arm (meters)
    std::map<std::string, double> momentArms;
};

// Millard2012 active force-length curve (normalized)
// All muscles use the same default parameters
static double activeForceLengthCurve(double normFiberLength) {
    // Gaussian-like curve centered at 1.0
    // Simplified Millard2012: bell curve from 0.5 to 1.5
    if (normFiberLength < 0.25 || normFiberLength > 1.9) return 0.0;

    // Piecewise approximation of the Millard curve
    double x = normFiberLength;
    if (x < 0.5) {
        return 0.0;
    } else if (x < 0.77) {
        // Shallow ascending
        double t = (x - 0.5) / (0.77 - 0.5);
        return 0.75 * t * t; // quadratic ramp
    } else if (x < 1.0) {
        // Steep ascending
        double t = (x - 0.77) / (1.0 - 0.77);
        return 0.75 * (1.0 - t * t) + t * t; // blend to 1.0
    } else if (x < 1.2) {
        // Peak plateau
        double t = (x - 1.0) / (1.2 - 1.0);
        return 1.0 - 0.1 * t; // slight decline
    } else {
        // Descending
        double t = (x - 1.2) / (1.9 - 1.2);
        return std::max(0.0, 0.9 * (1.0 - t));
    }
}

// Millard2012 force-velocity curve (normalized)
static double forceVelocityCurve(double normFiberVelocity) {
    // normFiberVelocity: -1 (max shortening) to +1 (max lengthening)
    // Returns force multiplier (0 at max shortening, 1 at isometric, ~1.4 at max lengthening)
    double v = std::clamp(normFiberVelocity, -1.0, 1.0);

    if (v <= 0) {
        // Concentric (shortening): Hill-type hyperbola
        // Simplified: linear ramp from 0 at v=-1 to 1 at v=0
        return 1.0 + v * (1.0 - 0.25 * v); // curved, not linear
    } else {
        // Eccentric (lengthening): approaches 1.4
        return 1.0 + v * 0.4 * (1.0 - 0.6 * v);
    }
}

// Compute muscle force given activation and normalized state
static double muscleForce(double activation, double normLength, double normVelocity,
                          const MuscleParams& params) {
    double fAL = activeForceLengthCurve(normLength);
    double fFV = forceVelocityCurve(normVelocity);
    double cosAlpha = std::cos(params.pennationAngleAtOptimal);

    return activation * params.maxIsometricForce * fAL * fFV * cosAlpha;
}

// Maximum force a muscle can produce at current state (when a=1)
static double maxMuscleForceAtState(double normLength, double normVelocity,
                                    const MuscleParams& params) {
    return muscleForce(1.0, normLength, normVelocity, params);
}

} // namespace muscle

// Forward declaration
static void assignMomentArmsFromName(muscle::MuscleParams& m);

// MARK: - .osim Muscle Parser

static std::vector<muscle::MuscleParams> parseMusclesFromOsim(const std::string& path) {
    std::vector<muscle::MuscleParams> muscles;

    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        NSLog(@"MuscleSolver: Failed to load .osim file");
        return muscles;
    }

    // Navigate to ForceSet
    auto* model = doc.FirstChildElement("OpenSimDocument");
    if (!model) model = doc.FirstChildElement("Model");
    if (model) {
        auto* modelEl = model->FirstChildElement("Model");
        if (modelEl) model = modelEl;
    }
    if (!model) return muscles;

    auto* forceSet = model->FirstChildElement("ForceSet");
    if (!forceSet) return muscles;
    auto* objects = forceSet->FirstChildElement("objects");
    if (!objects) return muscles;

    // Parse each Millard2012EquilibriumMuscle
    for (auto* muscleEl = objects->FirstChildElement("Millard2012EquilibriumMuscle");
         muscleEl;
         muscleEl = muscleEl->NextSiblingElement("Millard2012EquilibriumMuscle")) {

        muscle::MuscleParams params;
        params.maxContractionVelocity = 10.0; // default

        const char* name = muscleEl->Attribute("name");
        if (name) params.name = name;

        auto readDouble = [&](const char* tag) -> double {
            auto* el = muscleEl->FirstChildElement(tag);
            if (el && el->GetText()) return std::stod(el->GetText());
            return 0.0;
        };

        params.maxIsometricForce = readDouble("max_isometric_force");
        params.optimalFiberLength = readDouble("optimal_fiber_length");
        params.tendonSlackLength = readDouble("tendon_slack_length");
        params.pennationAngleAtOptimal = readDouble("pennation_angle_at_optimal");

        auto* vel = muscleEl->FirstChildElement("max_contraction_velocity");
        if (vel && vel->GetText()) params.maxContractionVelocity = std::stod(vel->GetText());

        // Parse GeometryPath to determine which joints the muscle crosses
        // For now, we use a simplified heuristic based on muscle name + attachment bodies
        assignMomentArmsFromName(params);

        if (params.maxIsometricForce > 0 && params.optimalFiberLength > 0) {
            muscles.push_back(params);
        }
    }

    return muscles;
}

// Heuristic moment arm assignment based on muscle name
// These are approximate values from the biomechanics literature
static void assignMomentArmsFromName(muscle::MuscleParams& m) {
    const std::string& n = m.name;

    // Hip flexors/extensors
    if (n.find("psoas") != std::string::npos || n.find("iliacus") != std::string::npos) {
        std::string side = (n.back() == 'r') ? "_r" : "_l";
        m.momentArms["hip_flexion" + side] = 0.03;
    }
    if (n.find("glmax") != std::string::npos || n.find("glmed") != std::string::npos || n.find("glmin") != std::string::npos) {
        std::string side = (n.back() == 'r') ? "_r" : "_l";
        m.momentArms["hip_flexion" + side] = -0.05;
        m.momentArms["hip_adduction" + side] = -0.03;
    }
    // Rectus femoris (hip flexor + knee extensor)
    if (n.find("recfem") != std::string::npos) {
        std::string side = (n.back() == 'r') ? "_r" : "_l";
        m.momentArms["hip_flexion" + side] = 0.04;
        m.momentArms["knee_angle" + side] = 0.04;
    }
    // Hamstrings (hip extensor + knee flexor)
    if (n.find("semimem") != std::string::npos || n.find("semiten") != std::string::npos ||
        n.find("bflh") != std::string::npos || n.find("bfsh") != std::string::npos) {
        std::string side = (n.back() == 'r') ? "_r" : "_l";
        m.momentArms["hip_flexion" + side] = -0.05;
        if (n.find("bfsh") == std::string::npos) // bfsh is short head, only crosses knee
            m.momentArms["hip_flexion" + side] = -0.05;
        m.momentArms["knee_angle" + side] = -0.03;
    }
    // Quadriceps (knee extensors)
    if (n.find("vasint") != std::string::npos || n.find("vaslat") != std::string::npos || n.find("vasmed") != std::string::npos) {
        std::string side = (n.back() == 'r') ? "_r" : "_l";
        m.momentArms["knee_angle" + side] = 0.04;
    }
    // Gastrocnemius (knee flexor + ankle plantarflexor)
    if (n.find("gaslat") != std::string::npos || n.find("gasmed") != std::string::npos) {
        std::string side = (n.back() == 'r') ? "_r" : "_l";
        m.momentArms["knee_angle" + side] = -0.02;
        m.momentArms["ankle_angle" + side] = -0.05;
    }
    // Soleus (ankle plantarflexor)
    if (n.find("soleus") != std::string::npos) {
        std::string side = (n.back() == 'r') ? "_r" : "_l";
        m.momentArms["ankle_angle" + side] = -0.05;
    }
    // Tibialis anterior (ankle dorsiflexor)
    if (n.find("tibant") != std::string::npos) {
        std::string side = (n.back() == 'r') ? "_r" : "_l";
        m.momentArms["ankle_angle" + side] = 0.04;
    }
    // Hip adductors
    if (n.find("addlong") != std::string::npos || n.find("addbrev") != std::string::npos || n.find("addmag") != std::string::npos) {
        std::string side = (n.back() == 'r') ? "_r" : "_l";
        m.momentArms["hip_adduction" + side] = 0.04;
        m.momentArms["hip_flexion" + side] = 0.02;
    }
    // Tensor fasciae latae
    if (n.find("tfl") != std::string::npos) {
        std::string side = (n.back() == 'r') ? "_r" : "_l";
        m.momentArms["hip_flexion" + side] = 0.03;
        m.momentArms["hip_adduction" + side] = -0.03;
    }
    // Piriformis and external rotators
    if (n.find("piri") != std::string::npos) {
        std::string side = (n.back() == 'r') ? "_r" : "_l";
        m.momentArms["hip_rotation" + side] = -0.03;
    }
    // Lumbar extensors
    if (n.find("ercspn") != std::string::npos || n.find("intobl") != std::string::npos || n.find("extobl") != std::string::npos) {
        m.momentArms["lumbar_extension"] = -0.05;
    }
    // Peroneus muscles (ankle evertors)
    if (n.find("perlong") != std::string::npos || n.find("perbrev") != std::string::npos) {
        std::string side = (n.back() == 'r') ? "_r" : "_l";
        m.momentArms["ankle_angle" + side] = -0.02;
    }
    // Gracilis
    if (n.find("grac") != std::string::npos) {
        std::string side = (n.back() == 'r') ? "_r" : "_l";
        m.momentArms["hip_adduction" + side] = 0.03;
        m.momentArms["knee_angle" + side] = -0.02;
    }
    // Sartorius
    if (n.find("sart") != std::string::npos) {
        std::string side = (n.back() == 'r') ? "_r" : "_l";
        m.momentArms["hip_flexion" + side] = 0.03;
        m.momentArms["knee_angle" + side] = -0.02;
    }
}

// MARK: - MuscleActivationResult

@implementation MuscleActivationResult {
    NSArray<NSString *> *_muscleNames;
    NSArray<NSNumber *> *_activations;
    NSArray<NSNumber *> *_forces;
    double _solveTimeMs;
    BOOL _converged;
}

- (instancetype)initWithNames:(NSArray<NSString *> *)names
                   activations:(NSArray<NSNumber *> *)activations
                        forces:(NSArray<NSNumber *> *)forces
                   solveTimeMs:(double)solveTimeMs
                     converged:(BOOL)converged {
    self = [super init];
    if (self) {
        _muscleNames = [names copy];
        _activations = [activations copy];
        _forces = [forces copy];
        _solveTimeMs = solveTimeMs;
        _converged = converged;
    }
    return self;
}

- (NSArray<NSString *> *)muscleNames { return _muscleNames; }
- (NSArray<NSNumber *> *)activations { return _activations; }
- (NSArray<NSNumber *> *)forces { return _forces; }
- (double)solveTimeMs { return _solveTimeMs; }
- (BOOL)converged { return _converged; }

@end

// MARK: - MuscleSolver

@implementation MuscleSolver {
    std::vector<muscle::MuscleParams> _muscles;
    OSQPSolver* _solver;
    BOOL _solverInitialized;

    // Previous activations for warm starting
    std::vector<double> _prevActivations;
}

- (instancetype)init {
    self = [super init];
    if (self) {
        _solver = nullptr;
        _solverInitialized = NO;
    }
    return self;
}

- (void)dealloc {
    if (_solver) {
        osqp_cleanup(_solver);
    }
}

- (BOOL)loadMusclesFromOsimPath:(NSString *)path {
    _muscles = parseMusclesFromOsim(std::string([path UTF8String]));
    _prevActivations.assign(_muscles.size(), 0.01);
    NSLog(@"MuscleSolver: Loaded %lu muscles", (unsigned long)_muscles.size());
    return !_muscles.empty();
}

- (NSInteger)numMuscles {
    return (NSInteger)_muscles.size();
}

- (NSArray<NSString *> *)muscleNames {
    NSMutableArray *names = [NSMutableArray array];
    for (const auto& m : _muscles) {
        [names addObject:[NSString stringWithUTF8String:m.name.c_str()]];
    }
    return names;
}

- (nullable MuscleActivationResult *)solveWithJointTorques:(NSArray<NSNumber *> *)jointTorques
                                               jointAngles:(NSArray<NSNumber *> *)jointAngles
                                           jointVelocities:(NSArray<NSNumber *> *)jointVelocities
                                                  dofNames:(NSArray<NSString *> *)dofNames {
    if (_muscles.empty()) return nil;

    double startTime = CACurrentMediaTime();

    NSInteger nMuscles = (NSInteger)_muscles.size();
    NSInteger nDOFs = dofNames.count;

    // Build DOF name to index map
    std::map<std::string, int> dofIndex;
    for (NSInteger i = 0; i < nDOFs; i++) {
        dofIndex[std::string([dofNames[i] UTF8String])] = (int)i;
    }

    // Compute max force at current state for each muscle (F_max * f_AL * f_FV * cos(alpha))
    // For now, use nominal values (normalized length = 1, velocity = 0)
    std::vector<double> maxForceAtState(nMuscles);
    for (NSInteger i = 0; i < nMuscles; i++) {
        // Simplified: assume normalized fiber length ~1.0 and velocity ~0
        // In a full implementation, we'd compute actual fiber length from skeleton pose
        double fAL = muscle::activeForceLengthCurve(1.0);
        double fFV = muscle::forceVelocityCurve(0.0);
        double cosAlpha = std::cos(_muscles[i].pennationAngleAtOptimal);
        maxForceAtState[i] = _muscles[i].maxIsometricForce * fAL * fFV * cosAlpha;
    }

    // Build the constraint matrix A and bounds
    // Constraint: R * diag(Fmax) * a = tau  (for each DOF)
    // Bounds: 0.01 <= a <= 1.0

    // Find which DOFs have muscles crossing them
    std::vector<int> activeDOFs;
    for (NSInteger d = 0; d < nDOFs; d++) {
        std::string dofName = std::string([dofNames[d] UTF8String]);
        bool hasMuscle = false;
        for (const auto& m : _muscles) {
            if (m.momentArms.count(dofName)) { hasMuscle = true; break; }
        }
        if (hasMuscle) activeDOFs.push_back((int)d);
    }

    if (activeDOFs.empty()) return nil;

    NSInteger nConstraints = (NSInteger)activeDOFs.size();
    NSInteger nTotal = nConstraints + 2 * nMuscles; // equality + upper + lower bounds

    // Build sparse A matrix in CSC format
    // A = [R*diag(Fmax); I; -I] where R is moment arm matrix
    // But OSQP uses: l <= Ax <= u format, so we stack:
    // Row 0..nConstraints-1: moment arm constraints (equality: l=u=tau)
    // Row nConstraints..nConstraints+nMuscles-1: upper bound (a <= 1)

    // For simplicity, use dense-to-sparse conversion
    // Matrix A has nTotal rows, nMuscles columns
    std::vector<double> A_dense(nTotal * nMuscles, 0.0);

    // Moment arm rows
    for (NSInteger c = 0; c < nConstraints; c++) {
        int dofIdx = activeDOFs[c];
        std::string dofName = std::string([dofNames[dofIdx] UTF8String]);

        for (NSInteger m = 0; m < nMuscles; m++) {
            auto it = _muscles[m].momentArms.find(dofName);
            if (it != _muscles[m].momentArms.end()) {
                // A[c, m] = moment_arm * max_force_at_state
                A_dense[c + m * nTotal] = it->second * maxForceAtState[m];
            }
        }
    }

    // Identity rows for upper bounds (a <= 1)
    for (NSInteger m = 0; m < nMuscles; m++) {
        A_dense[(nConstraints + m) + m * nTotal] = 1.0;
    }

    // Convert to CSC
    std::vector<OSQPInt> A_col_ptr(nMuscles + 1, 0);
    std::vector<OSQPInt> A_row_idx;
    std::vector<OSQPFloat> A_val;

    for (NSInteger col = 0; col < nMuscles; col++) {
        A_col_ptr[col] = (OSQPInt)A_row_idx.size();
        for (NSInteger row = 0; row < nTotal; row++) {
            double v = A_dense[row + col * nTotal];
            if (std::abs(v) > 1e-12) {
                A_row_idx.push_back((OSQPInt)row);
                A_val.push_back(v);
            }
        }
    }
    A_col_ptr[nMuscles] = (OSQPInt)A_row_idx.size();

    // P matrix: identity (min sum(a^2))
    std::vector<OSQPInt> P_col_ptr(nMuscles + 1);
    std::vector<OSQPInt> P_row_idx(nMuscles);
    std::vector<OSQPFloat> P_val(nMuscles, 1.0);
    for (NSInteger i = 0; i < nMuscles; i++) {
        P_col_ptr[i] = (OSQPInt)i;
        P_row_idx[i] = (OSQPInt)i;
    }
    P_col_ptr[nMuscles] = (OSQPInt)nMuscles;

    // q vector: zeros (no linear cost)
    std::vector<OSQPFloat> q(nMuscles, 0.0);

    // Bounds
    std::vector<OSQPFloat> l(nTotal), u(nTotal);
    // Equality constraints (moment arm = tau)
    for (NSInteger c = 0; c < nConstraints; c++) {
        int dofIdx = activeDOFs[c];
        double tau = [jointTorques[dofIdx] doubleValue];
        l[c] = tau;
        u[c] = tau;
    }
    // Activation bounds: 0.01 <= a <= 1.0
    for (NSInteger m = 0; m < nMuscles; m++) {
        l[nConstraints + m] = 0.01;
        u[nConstraints + m] = 1.0;
    }

    // Setup OSQP
    OSQPCscMatrix P_csc, A_csc;
    OSQPCscMatrix_set_data(&P_csc, (OSQPInt)nMuscles, (OSQPInt)nMuscles,
                 (OSQPInt)P_val.size(), P_val.data(), P_row_idx.data(), P_col_ptr.data());
    OSQPCscMatrix_set_data(&A_csc, (OSQPInt)nTotal, (OSQPInt)nMuscles,
                 (OSQPInt)A_val.size(), A_val.data(), A_row_idx.data(), A_col_ptr.data());

    // Cleanup previous solver if problem size changed
    if (_solver) {
        osqp_cleanup(_solver);
        _solver = nullptr;
    }

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.max_iter = 200;
    settings.eps_abs = 1e-3;
    settings.eps_rel = 1e-3;
    settings.verbose = false;
    settings.warm_starting = true;
    settings.polishing = false;

    OSQPInt exitflag = osqp_setup(&_solver, &P_csc, q.data(), &A_csc,
                                   l.data(), u.data(), (OSQPInt)nTotal, (OSQPInt)nMuscles,
                                   &settings);
    if (exitflag != 0 || !_solver) {
        NSLog(@"MuscleSolver: OSQP setup failed with code %d", (int)exitflag);
        return nil;
    }

    // Warm start with previous activations
    if (_prevActivations.size() == (size_t)nMuscles) {
        osqp_warm_start(_solver, _prevActivations.data(), nullptr);
    }

    // Solve
    osqp_solve(_solver);

    BOOL converged = (_solver->info->status_val == OSQP_SOLVED ||
                      _solver->info->status_val == OSQP_SOLVED_INACCURATE);

    // Extract results
    NSMutableArray<NSString *> *names = [NSMutableArray arrayWithCapacity:nMuscles];
    NSMutableArray<NSNumber *> *activations = [NSMutableArray arrayWithCapacity:nMuscles];
    NSMutableArray<NSNumber *> *forces = [NSMutableArray arrayWithCapacity:nMuscles];

    for (NSInteger i = 0; i < nMuscles; i++) {
        double a = converged ? std::clamp(_solver->solution->x[i], 0.01, 1.0) : 0.01;
        double f = a * maxForceAtState[i];

        [names addObject:[NSString stringWithUTF8String:_muscles[i].name.c_str()]];
        [activations addObject:@(a)];
        [forces addObject:@(f)];

        _prevActivations[i] = a;
    }

    double solveTime = (CACurrentMediaTime() - startTime) * 1000.0;

    osqp_cleanup(_solver);
    _solver = nullptr;

    return [[MuscleActivationResult alloc] initWithNames:names
                                             activations:activations
                                                  forces:forces
                                             solveTimeMs:solveTime
                                               converged:converged];
}

@end
