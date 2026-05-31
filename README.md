# kinova-gen3-trajectory

> **MATLAB implementation of a complete robot motion pipeline for the Kinova Gen3 7-DOF arm** —
> custom Damped Least-Squares IK solver, cubic polynomial joint-space trajectory planner, and live animation. No toolbox IK objects required.

---

## What This Does

Given a list of Cartesian waypoints, this pipeline:

1. Solves inverse kinematics for each waypoint using a custom **Damped Least-Squares (DLS)** iterative solver built directly on `geometricJacobian` and `getTransform`
2. Blends the resulting joint configurations into a smooth **cubic polynomial trajectory** with C¹ continuity (positions and velocities match at every waypoint boundary)
3. Animates the full motion using MATLAB's Robotics System Toolbox visualiser

---

## Requirements

| Dependency | Version |
|---|---|
| MATLAB | R2021b or later |
| Robotics System Toolbox | Bundled with MATLAB |

No third-party packages or additional toolboxes are needed.

---

## Quick Start

```matlab
% 1. Clone the repository
% 2. Open MATLAB and navigate to the repo root
cd kinova-gen3-trajectory

% 3. Run the entry point
main
```

The script will:
- Load the `kinovaGen3` rigid body model
- Solve IK for 5 Cartesian waypoints, printing `OK` / `POOR` per waypoint
- Build a smooth cubic trajectory across all waypoints
- Open an animation window and play the motion

Expected console output:
```
Solving IK for 5 waypoints...
------------------------------------------
  Waypoint  1:  OK    err = 0.0031 m
  Waypoint  2:  OK    err = 0.0048 m
  Waypoint  3:  OK    err = 0.0062 m
  Waypoint  4:  OK    err = 0.0027 m
  Waypoint  5:  OK    err = 0.0031 m
------------------------------------------

Animating 240 frames...
Done.
```

---

## Repository Structure

```
kinova-gen3-trajectory/
├── main.m                   ← Entry point — run this
│
├── config/
│   └── params.m             ← All tunable parameters in one place
│
├── ik/
│   └── basicIK.m            ← Damped Jacobian IK solver (standalone function)
│
├── trajectory/
│   └── cubicTraj.m          ← Cubic polynomial trajectory generator
│
├── utils/
│   ├── clampJoints.m        ← Joint-limit clamping utility
│   └── checkIKError.m       ← Per-waypoint error reporting
│
├── tests/
│   ├── test_basicIK.m       ← 5 unit tests for the IK solver
│   └── test_cubicTraj.m     ← 6 unit tests for the trajectory generator
│
└── docs/
    └── algorithm_notes.md   ← Full mathematical derivations (DLS-IK + cubic poly)
```

---

## Configuration

All tunable parameters live in `config/params.m`. Edit this file to adjust the solver and trajectory without touching any function code.

```matlab
% --- IK Solver ---
ikParams.maxIter = 500;    % Maximum Newton-Raphson iterations
ikParams.alpha   = 0.5;    % Step size (0 < alpha ≤ 1)
ikParams.tol     = 1e-3;   % Convergence tolerance on 6D error norm
ikParams.lambda  = 0.01;   % DLS damping coefficient

% --- Trajectory ---
FRAMES_PER_SEG = 60;       % Animation frames per trajectory segment
SEG_DURATION   = 1.0;      % Duration of each segment in seconds

% --- Acceptance ---
POS_ERR_THRESH = 0.01;     % Metres — below this is "OK", above is "POOR"
```

### Tuning the IK Solver

| Parameter | Effect |
|---|---|
| `alpha` ↑ | Faster convergence, risk of oscillation |
| `alpha` ↓ | Slower convergence, more stable near singularities |
| `lambda` ↑ | More damping, better singularity handling, lower accuracy |
| `lambda` ↓ | Less damping, higher accuracy, less stable at singularities |
| `maxIter` ↑ | More chances to converge on difficult poses |

---

## Waypoints

The default waypoints are defined in `main.m` and can be freely edited:

| # | x (m) | y (m) | z (m) | Notes |
|---|---|---|---|---|
| 1 | 0.40 | 0.00 | 0.50 | Start / end pose |
| 2 | 0.40 | 0.20 | 0.60 | Forward-left, raised |
| 3 | 0.20 | 0.30 | 0.40 | Further left, lower |
| 4 | 0.30 | −0.20 | 0.50 | Right side |
| 5 | 0.40 | 0.00 | 0.50 | Return to start |

The end-effector orientation is fixed to **pointing straight down** (`eul2rotm([0, π, 0], 'ZYX')`) for all waypoints. Orientation interpolation between waypoints is not currently implemented — see [Known Limitations](#known-limitations).

---

## Running the Tests

```matlab
cd kinova-gen3-trajectory

% IK solver — 5 checks
run('tests/test_basicIK.m')

% Trajectory generator — 6 checks
run('tests/test_cubicTraj.m')
```

Each test prints `PASS` or `FAIL` with the measured value. Tests cover output dimensions, convergence accuracy, joint limit enforcement, and trajectory continuity.

---

## Algorithm Overview

### Damped Least-Squares Inverse Kinematics

The solver (`ik/basicIK.m`) iterates a Newton-Raphson update using a damped pseudoinverse Jacobian. The 6D task-space error is:

```
e = [ e_orientation ]   where  e_ori = 0.5 · vex(R_target · R_cur')
    [ e_position    ]          e_pos = p_target - p_cur
```

The damped pseudoinverse Jacobian is:

```
J_dls = Jᵀ (J Jᵀ + λ²I)⁻¹
```

And the joint update per iteration is:

```
dq = α · J_dls · e
q  ← q + dq    (then clamped to joint limits)
```

The damping coefficient `λ` prevents numerical blow-up near kinematic singularities. See [`docs/algorithm_notes.md`](docs/algorithm_notes.md) for the full derivation.

---

### Cubic Polynomial Trajectory

The trajectory generator (`trajectory/cubicTraj.m`) fits a cubic polynomial over each joint-space segment:

```
q(t) = a₀ + a₁t + a₂t² + a₃t³     t ∈ [0, T]
```

Coefficients are solved from four boundary conditions — position and velocity at each segment endpoint:

```
a₀ =  q₀
a₁ =  q̇₀
a₂ =  (3/T²)(q₁ − q₀) − (2/T)q̇₀ − (1/T)q̇₁
a₃ = (−2/T³)(q₁ − q₀) + (1/T²)(q̇₀ + q̇₁)
```

Interior waypoint velocities are estimated via **central differences**:

```
q̇ₖ = (qₖ₊₁ − qₖ₋₁) / (2T)
```

This gives **C¹ continuity** — positions and velocities are matched at every waypoint boundary. Endpoint velocities are set to zero (natural boundary condition).

The function also returns analytic velocity (`qdFull`) and acceleration (`qddFull`) arrays for downstream use.

---

## Known Limitations

- **Fixed orientation** — the end-effector always points downward; there is no orientation interpolation between waypoints.
- **No collision detection** — no obstacle avoidance or self-collision checking is performed.
- **Velocity/acceleration limits not enforced** — only joint position limits are clamped. The trajectory may exceed hardware velocity or torque limits for fast segments.
- **C¹ continuity only** — accelerations are discontinuous at waypoint boundaries. For C² continuity, replace the central-difference velocity estimates with a natural cubic spline (requires solving a tridiagonal system across all waypoints simultaneously). See [`docs/algorithm_notes.md`](docs/algorithm_notes.md) for details.
- **Fixed warm-start** — the IK solver always initialises from the home configuration, not the previous waypoint solution. Warm-starting from the previous solution would improve convergence speed and consistency across closely-spaced waypoints.

---

## File Reference

| File | Purpose |
|---|---|
| `main.m` | Entry point — loads params, runs IK loop, builds trajectory, animates |
| `config/params.m` | Central config for all tunable parameters |
| `ik/basicIK.m` | DLS iterative IK solver; inputs: robot, EEF name, target pose, params struct |
| `trajectory/cubicTraj.m` | Cubic trajectory generator; outputs: positions, velocities, accelerations |
| `utils/clampJoints.m` | Clamps a joint config to each joint's `PositionLimits` |
| `utils/checkIKError.m` | Computes and prints Euclidean position error for a solved IK result |
| `tests/test_basicIK.m` | Unit tests: output size, convergence, joint limits, warm/cold start |
| `tests/test_cubicTraj.m` | Unit tests: dimensions, endpoint matching, continuity, no large jumps |
| `docs/algorithm_notes.md` | Mathematical derivations for DLS-IK and cubic polynomial trajectory |

---

## License

MIT — see [LICENSE](LICENSE).