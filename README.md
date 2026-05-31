# Kinova Gen3 — Waypoint IK + Cubic Polynomial Trajectory

MATLAB implementation of a full robot motion pipeline for the **Kinova Gen3 7-DOF** arm:

- **Custom iterative IK solver** (Damped Least-Squares, no toolbox IK objects)
- **Cubic polynomial joint-space trajectory** with central-difference velocity blending
- **Live animation** via MATLAB's Robotics System Toolbox visualiser

---

## Repository Structure

```
kinova-gen3-trajectory/
├── main.m                  ← Entry point — run this
│
├── config/
│   └── params.m            ← All tunable parameters in one place
│
├── ik/
│   └── basicIK.m           ← Damped Jacobian IK solver (standalone function)
│
├── trajectory/
│   └── cubicTraj.m         ← Cubic polynomial trajectory generator
│
├── utils/
│   ├── clampJoints.m       ← Joint limit clamping
│   └── checkIKError.m      ← Waypoint error reporting
│
├── tests/
│   ├── test_basicIK.m      ← Unit tests for the IK solver
│   └── test_cubicTraj.m    ← Unit tests for the trajectory generator
│
└── docs/
    └── algorithm_notes.md  ← DLS-IK and cubic polynomial derivations
```

---

## Dependencies

| Requirement | Version |
|-------------|---------|
| MATLAB | R2021b or later |
| Robotics System Toolbox | included with MATLAB |

No additional toolboxes or packages required.

---

## Quick Start

```matlab
% 1. Clone or download the repo
% 2. Open MATLAB and cd into the repo root
cd kinova-gen3-trajectory

% 3. Run
main
```

The script will:
1. Load the Kinova Gen3 model
2. Solve IK for 5 Cartesian waypoints (prints OK/POOR per waypoint)
3. Build a smooth cubic trajectory
4. Open an animation window

---

## Configuration

All tunable parameters live in `config/params.m`:

```matlab
% IK Solver
ikParams.maxIter = 500;    % max Newton-Raphson iterations
ikParams.alpha   = 0.5;    % step size  (0 < alpha ≤ 1)
ikParams.tol     = 1e-3;   % convergence tolerance
ikParams.lambda  = 0.01;   % DLS damping coefficient

% Trajectory
FRAMES_PER_SEG = 60;       % animation frames per segment
SEG_DURATION   = 1.0;      % seconds per segment

% Acceptance
POS_ERR_THRESH = 0.01;     % metres — below this is "OK"
```

Edit this file to tune the solver without touching any function code.

---

## Running the Tests

```matlab
cd kinova-gen3-trajectory

% IK solver tests (5 checks)
run('tests/test_basicIK.m')

% Trajectory tests (6 checks)
run('tests/test_cubicTraj.m')
```

Each test prints `PASS` or `FAIL` with the measured error.

---

## Algorithm Overview

### Damped Least-Squares IK

The solver iterates a Newton-Raphson update with a damped pseudoinverse Jacobian:

```
J_dls = Jᵀ (J Jᵀ + λ²I)⁻¹
dq    = α · J_dls · e
```

where `e = [e_orientation ; e_position]` is the 6D task-space error.
Damping coefficient `λ` prevents blow-up near singularities.

### Cubic Polynomial Trajectory

Each joint-space segment uses:

```
q(t) = a₀ + a₁t + a₂t² + a₃t³
```

Boundary velocities at interior waypoints are estimated by central differences.  This gives C1 continuity (positions and velocities match at waypoint boundaries).

See [`docs/algorithm_notes.md`](docs/algorithm_notes.md) for full derivations.

---

## Waypoints

The default waypoints (editable in `main.m`):

| # | x (m) | y (m) | z (m) |
|---|-------|-------|-------|
| 1 | 0.40  |  0.00 | 0.50  |
| 2 | 0.40  |  0.20 | 0.60  |
| 3 | 0.20  |  0.30 | 0.40  |
| 4 | 0.30  | −0.20 | 0.50  |
| 5 | 0.40  |  0.00 | 0.50  |

The end-effector orientation is fixed as **pointing downward** (`eul2rotm([0, π, 0], 'ZYX')`).

---

## Known Limitations

- Orientation is fixed (downward); no orientation interpolation between waypoints.
- No collision detection or self-collision avoidance.
- Joint velocity/acceleration limits are not enforced in the trajectory (only position limits).
- C2 continuity (smooth acceleration) requires a full natural cubic spline — see `docs/algorithm_notes.md` for details.

---

## License

MIT — see [LICENSE](LICENSE).
