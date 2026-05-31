function qSol = basicIK(robot, endEffector, targetPose, params)
%BASICIK  Damped Least-Squares (DLS) iterative inverse kinematics solver.
%
%  Solves the IK problem using a Newton-Raphson loop with a damped
%  pseudoinverse Jacobian.  No toolbox IK objects are used — only
%  getTransform() and geometricJacobian() from the Robotics System Toolbox.
%
%  Algorithm
%  ---------
%    1. Forward kinematics → current EEF pose T_cur
%    2. 6D pose error:
%         e_pos = p_target - p_cur
%         e_ori = 0.5 * skew_vex(R_target * R_cur')
%         err   = [e_ori; e_pos]
%    3. Geometric Jacobian J at current config
%    4. Damped pseudoinverse:
%         J_dls = J' * inv(J*J' + λ²·I)
%    5. Joint update:
%         dq = α * J_dls * err
%         q  = q + dq   (clamped to joint limits)
%    6. Repeat until ||err|| < tol or maxIter reached
%
%  Inputs
%  ------
%    robot        : rigidBodyTree robot model (DataFormat = "row")
%    endEffector  : name of the end-effector body (string)
%    targetPose   : 4×4 homogeneous transformation (desired EEF pose)
%    params       : struct with fields:
%                     .maxIter  – max iterations           (default 500)
%                     .alpha    – step size                (default 0.5)
%                     .tol      – convergence tolerance    (default 1e-3)
%                     .lambda   – DLS damping coefficient  (default 0.01)
%
%  Output
%  ------
%    qSol  : 1×N joint configuration (row vector, radians)
%
%  Example
%  -------
%    gen3  = loadrobot("kinovaGen3", DataFormat="row");
%    T_tgt = trvec2tform([0.4, 0.0, 0.5]) * rotm2tform(eul2rotm([0,pi,0],'ZYX'));
%    p.maxIter = 500; p.alpha = 0.5; p.tol = 1e-3; p.lambda = 0.01;
%    q = basicIK(gen3, 'EndEffector_Link', T_tgt, p);
%
%  See also: geometricJacobian, getTransform, homeConfiguration

% ---- defaults -----------------------------------------------------------
if nargin < 4 || isempty(params)
    params = struct();
end
maxIter = getfield_default(params, 'maxIter', 500);
alpha   = getfield_default(params, 'alpha',   0.5);
tol     = getfield_default(params, 'tol',     1e-3);
lambda  = getfield_default(params, 'lambda',  0.01);

% ---- initialise from home config ----------------------------------------
q = homeConfiguration(robot);

% ---- main loop ----------------------------------------------------------
for iter = 1:maxIter

    % Forward kinematics
    T_cur = getTransform(robot, q, endEffector);

    % --- Position error ---
    pos_err = targetPose(1:3, 4) - T_cur(1:3, 4);

    % --- Orientation error (axis-angle from residual rotation) ---
    R_cur    = T_cur(1:3, 1:3);
    R_target = targetPose(1:3, 1:3);
    R_err    = R_target * R_cur';
    ori_err  = 0.5 * [R_err(3,2) - R_err(2,3);
                      R_err(1,3) - R_err(3,1);
                      R_err(2,1) - R_err(1,2)];

    % --- Full 6D error [orientation; position] ---
    err = [ori_err; pos_err];

    if norm(err) < tol
        break;
    end

    % --- Geometric Jacobian ---
    J = geometricJacobian(robot, q, endEffector);

    % --- Damped least-squares pseudoinverse ---
    J_dls = J' / (J * J' + lambda^2 * eye(6));

    % --- Joint update ---
    dq = alpha * (J_dls * err)';
    q  = q + dq;

    % --- Clamp to joint limits ---
    q = clampJoints(robot, q);
end

qSol = q;
end

% =========================================================================
%  Private helper — safe struct field access with default
% =========================================================================
function val = getfield_default(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end
