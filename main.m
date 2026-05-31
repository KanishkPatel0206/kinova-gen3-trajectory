%% Kinova Gen3 — Waypoint IK + Cubic Polynomial Trajectory
%
%  Entry point. Loads parameters, solves IK for each Cartesian waypoint,
%  builds a cubic polynomial joint-space trajectory, and animates the result.
%
%  Dependencies:
%    - MATLAB Robotics System Toolbox (R2021b+)
%    - config/params.m
%    - ik/basicIK.m
%    - trajectory/cubicTraj.m
%    - utils/checkIKError.m
%
%  Usage:
%    >> main
%
%  See also: ik/basicIK.m, trajectory/cubicTraj.m, config/params.m

clear; clc; close all;

%% 0. Load configuration
run('config/params.m');

%% 1. Load robot model
gen3        = loadrobot("kinovaGen3", DataFormat="row");
endEffector = 'EndEffector_Link';

%% 2. Define Cartesian waypoints  [x, y, z]  in metres
waypoints = [
    0.4,  0.0,  0.5;
    0.4,  0.2,  0.6;
    0.2,  0.3,  0.4;
    0.3, -0.2,  0.5;
    0.4,  0.0,  0.5;
];

%% 3. Fixed end-effector orientation (pointing downward)
R_down         = eul2rotm([0, pi, 0], 'ZYX');
T_down         = rotm2tform(R_down);

%% 4. Solve IK for each waypoint
numWaypoints = size(waypoints, 1);
qWaypoints   = zeros(numWaypoints, 7);
q0           = homeConfiguration(gen3);

fprintf('\nSolving IK for %d waypoints...\n', numWaypoints);
fprintf('%s\n', repmat('-', 1, 42));

for i = 1:numWaypoints
    % Build target pose
    targetPose         = T_down;
    targetPose(1:3, 4) = waypoints(i, :)';

    % Warm-start from previous solution
    if i == 1
        guess = q0;
    else
        guess = qWaypoints(i-1, :);
    end

    % Solve
    qSol = basicIK(gen3, endEffector, targetPose, ikParams);

    % Report
    checkIKError(gen3, endEffector, qSol, waypoints(i, :), i, POS_ERR_THRESH);

    qWaypoints(i, :) = qSol;
end

fprintf('%s\n\n', repmat('-', 1, 42));

%% 5. Build cubic polynomial trajectory
[qFull, ~, ~] = cubicTraj(qWaypoints, FRAMES_PER_SEG, SEG_DURATION);

%% 6. Animate
fprintf('Animating %d frames...\n', size(qFull, 1));
figure('Name', 'Kinova Gen3 — IK + Cubic Trajectory', 'Color', 'w');

totalFrames = size(qFull, 1);
for i = 1:totalFrames
    show(gen3, qFull(i, :), FastUpdate=true, PreservePlot=false);
    seg = ceil(i / FRAMES_PER_SEG);
    title(sprintf('Segment %d/%d   |   Frame %d/%d', ...
        seg, numWaypoints - 1, i, totalFrames), FontSize=11);
    drawnow;
    pause(0.02);
end

fprintf('Done.\n');
