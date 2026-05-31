# Algorithm Notes

## 1. Damped Least-Squares Inverse Kinematics

### Problem Statement

Given a desired end-effector pose $T_{\text{target}} \in SE(3)$, find joint angles $\mathbf{q} \in \mathbb{R}^n$ such that:

$$FK(\mathbf{q}) = T_{\text{target}}$$

where $FK$ is the forward kinematics map.

### 6D Pose Error

The error vector has two parts:

**Position error:**
$$\mathbf{e}_p = \mathbf{p}_{\text{target}} - \mathbf{p}_{\text{current}}$$

**Orientation error** (from the skew-symmetric part of the residual rotation):
$$R_{\text{err}} = R_{\text{target}} \cdot R_{\text{current}}^T$$
$$\mathbf{e}_o = \frac{1}{2} \begin{bmatrix} R_{\text{err},32} - R_{\text{err},23} \\ R_{\text{err},13} - R_{\text{err},31} \\ R_{\text{err},21} - R_{\text{err},12} \end{bmatrix}$$

Full error: $\mathbf{e} = [\mathbf{e}_o;\, \mathbf{e}_p] \in \mathbb{R}^6$

### Damped Least-Squares (DLS) Update

The standard pseudoinverse $J^+$ becomes ill-conditioned near singularities.  DLS regularises it:

$$J_{\text{DLS}} = J^T (J J^T + \lambda^2 I)^{-1}$$

Joint update:
$$\Delta \mathbf{q} = \alpha \cdot J_{\text{DLS}} \cdot \mathbf{e}$$
$$\mathbf{q} \leftarrow \mathbf{q} + \Delta \mathbf{q}$$

**Parameters:**

| Symbol | Name | Typical value |
|--------|------|---------------|
| $\alpha$ | Step size | 0.5 |
| $\lambda$ | Damping coefficient | 0.01 |
| $\epsilon$ | Convergence tolerance | $10^{-3}$ |

**Convergence criterion:** $\|\mathbf{e}\| < \epsilon$

### Notes

- The DLS damping trades off accuracy near singularities for numerical stability.  Larger $\lambda$ → more damping → slower convergence.
- $\alpha < 1$ prevents oscillation; the algorithm is essentially gradient descent on the task-space error.
- Joint limits are enforced by clamping after each update (`utils/clampJoints.m`).  This can slow convergence near the workspace boundary.

---

## 2. Cubic Polynomial Trajectory

### Problem Statement

Given $M$ joint-space waypoints $\mathbf{q}_0, \ldots, \mathbf{q}_{M-1}$ and desired boundary velocities, generate a smooth trajectory that passes exactly through each waypoint.

### Segment Parameterisation

Each segment $k \in \{0, \ldots, M-2\}$ is parameterised over $t \in [0, T]$:

$$\mathbf{q}(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3$$

### Boundary Conditions

At $t = 0$: $\mathbf{q}(0) = \mathbf{q}_k$, $\dot{\mathbf{q}}(0) = \dot{\mathbf{q}}_k$

At $t = T$: $\mathbf{q}(T) = \mathbf{q}_{k+1}$, $\dot{\mathbf{q}}(T) = \dot{\mathbf{q}}_{k+1}$

Solving the 4×4 linear system yields:

$$a_0 = \mathbf{q}_k$$
$$a_1 = \dot{\mathbf{q}}_k$$
$$a_2 = \frac{3}{T^2}(\mathbf{q}_{k+1} - \mathbf{q}_k) - \frac{2}{T}\dot{\mathbf{q}}_k - \frac{1}{T}\dot{\mathbf{q}}_{k+1}$$
$$a_3 = \frac{-2}{T^3}(\mathbf{q}_{k+1} - \mathbf{q}_k) + \frac{1}{T^2}(\dot{\mathbf{q}}_k + \dot{\mathbf{q}}_{k+1})$$

### Velocity Estimation (Central Differences)

For interior waypoints:

$$\dot{\mathbf{q}}_k = \frac{\mathbf{q}_{k+1} - \mathbf{q}_{k-1}}{2T}$$

Endpoint velocities are set to zero (natural boundary condition).

### Derivatives

The analytic velocity and acceleration are used by `cubicTraj.m`:

$$\dot{\mathbf{q}}(t) = a_1 + 2a_2 t + 3a_3 t^2$$
$$\ddot{\mathbf{q}}(t) = 2a_2 + 6a_3 t$$

### Properties

- **Positional continuity (C0):** guaranteed by construction (endpoints match).
- **Velocity continuity (C1):** enforced at interior waypoints via central-difference estimates.
- **Acceleration:** continuous within each segment, but discontinuous at waypoint boundaries (C1 trajectory, not C2).
- For C2 continuity, use a natural cubic spline (solve a tridiagonal system for all velocities simultaneously).

---

## 3. Kinova Gen3 — Key Facts

| Property | Value |
|----------|-------|
| DOF | 7 |
| Payload | 2 kg |
| Reach | 902 mm |
| End-effector | `EndEffector_Link` |
| Joint types | All revolute |
| MATLAB model | `loadrobot("kinovaGen3")` |

The 7th DOF (wrist rotation) gives the robot a redundant degree of freedom for a 6D task, which the DLS-IK exploits automatically through the pseudoinverse.
