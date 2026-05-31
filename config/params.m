%% config/params.m
%
%  Central configuration for all tunable parameters.
%  Run this script (via `run('config/params.m')`) before calling any solver
%  or trajectory function.
%
%  All IK solver settings are bundled into the `ikParams` struct so they
%  can be passed cleanly to basicIK() without long argument lists.

% -------------------------------------------------------------------------
%  IK Solver
% -------------------------------------------------------------------------
ikParams.maxIter   = 500;    % Maximum Newton-Raphson iterations
ikParams.alpha     = 0.5;    % Step size (damping factor, 0 < alpha <= 1)
ikParams.tol       = 1e-3;   % Convergence tolerance on 6D error norm
ikParams.lambda    = 0.01;   % DLS regularisation coefficient

% -------------------------------------------------------------------------
%  Trajectory
% -------------------------------------------------------------------------
FRAMES_PER_SEG = 60;    % Animation frames per trajectory segment
SEG_DURATION   = 1.0;   % Duration of each segment in seconds

% -------------------------------------------------------------------------
%  Acceptance threshold
% -------------------------------------------------------------------------
POS_ERR_THRESH = 0.01;  % metres — waypoint is flagged POOR above this
