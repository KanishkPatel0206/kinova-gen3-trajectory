function [qFull, qdFull, qddFull] = cubicTraj(qWaypoints, framesPerSeg, T)
%CUBICTRAJ  Cubic polynomial joint-space trajectory through waypoints.
%
%  Generates a smooth joint-space trajectory that passes through each
%  waypoint configuration.  Boundary velocities are estimated using
%  central differences; endpoint velocities are set to zero.
%
%  Cubic polynomial form (per segment, per joint):
%
%    q(t) = a0 + a1*t + a2*t^2 + a3*t^3
%
%  Coefficients derived from boundary conditions:
%    q(0)  = q0,  q(T)  = q1
%    q'(0) = qd0, q'(T) = qd1
%
%  Inputs
%  ------
%    qWaypoints   : M×N matrix of joint configurations (M waypoints, N DOF)
%    framesPerSeg : number of sample frames per segment        (default 60)
%    T            : duration of each segment in seconds        (default 1.0)
%
%  Outputs
%  -------
%    qFull   : (M-1)*framesPerSeg × N  — joint positions
%    qdFull  : (M-1)*framesPerSeg × N  — joint velocities
%    qddFull : (M-1)*framesPerSeg × N  — joint accelerations
%
%  Example
%  -------
%    qWP = [q1; q2; q3; q4];          % 4×7 for a 7-DOF robot
%    [q, qd, qdd] = cubicTraj(qWP, 60, 1.0);
%    plot(q);
%
%  See also: basicIK, main

% ---- defaults -----------------------------------------------------------
if nargin < 2 || isempty(framesPerSeg), framesPerSeg = 60;  end
if nargin < 3 || isempty(T),            T            = 1.0; end

[numWP, nDOF] = size(qWaypoints);
numSegs       = numWP - 1;
totalFrames   = numSegs * framesPerSeg;

qFull   = zeros(totalFrames, nDOF);
qdFull  = zeros(totalFrames, nDOF);
qddFull = zeros(totalFrames, nDOF);

% ---- Velocity estimates at waypoints (central differences) --------------
qd = zeros(numWP, nDOF);
for i = 2:numWP - 1
    qd(i, :) = 0.5 * (qWaypoints(i+1, :) - qWaypoints(i-1, :)) / T;
end
% Endpoint velocities remain zero (natural boundary condition)

% ---- Build each segment -------------------------------------------------
t_vec = linspace(0, T, framesPerSeg);
row   = 1;

for seg = 1:numSegs
    q0s  = qWaypoints(seg,   :);
    q1s  = qWaypoints(seg+1, :);
    qd0s = qd(seg,   :);
    qd1s = qd(seg+1, :);

    % Cubic coefficients
    a0 =  q0s;
    a1 =  qd0s;
    a2 =  (3/T^2) .* (q1s - q0s) - (2/T) .* qd0s - (1/T) .* qd1s;
    a3 = (-2/T^3) .* (q1s - q0s) + (1/T^2) .* (qd0s + qd1s);

    for k = 1:framesPerSeg
        t_k = t_vec(k);

        qFull(row, :)   =          a0 +     a1*t_k +     a2*t_k^2 +     a3*t_k^3;
        qdFull(row, :)  =               a1 +  2*a2*t_k +  3*a3*t_k^2;
        qddFull(row, :) =                      2*a2   +  6*a3*t_k;

        row = row + 1;
    end
end
end
