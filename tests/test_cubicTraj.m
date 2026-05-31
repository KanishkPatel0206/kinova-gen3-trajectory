%% tests/test_cubicTraj.m
%
%  Unit tests for the cubicTraj() function.
%
%  Checks:
%    1. Output dimensions match (numSegs * framesPerSeg) × nDOF
%    2. Trajectory hits first waypoint exactly at t=0
%    3. Trajectory hits last waypoint exactly at t=T
%    4. Velocity output is correct size
%    5. Acceleration output is correct size
%    6. Continuity — no large jumps between consecutive frames
%
%  Usage:
%    >> cd <repo-root>
%    >> run('tests/test_cubicTraj.m')

clear; clc;
addpath('trajectory', 'config');
run('config/params.m');

fprintf('\n=== cubicTraj unit tests ===\n\n');

PASS = 0; FAIL = 0;

% Simple synthetic waypoints (3 DOF for clarity)
qWP = [0,  0,  0;
       1,  2, -1;
       0,  1,  1;
       2, -1,  0];

framesPerSeg = FRAMES_PER_SEG;
T            = SEG_DURATION;
nDOF         = size(qWP, 2);
numWP        = size(qWP, 1);
numSegs      = numWP - 1;

[qFull, qdFull, qddFull] = cubicTraj(qWP, framesPerSeg, T);

% -------------------------------------------------------------------------
% Test 1 — Output size
% -------------------------------------------------------------------------
expectedRows = numSegs * framesPerSeg;
ok = isequal(size(qFull), [expectedRows, nDOF]);
if ok, PASS = PASS+1; else, FAIL = FAIL+1; end
if ok
    fprintf('  PASS  Output size: [%d × %d]\n', size(qFull,1), size(qFull,2));
else
    fprintf('  FAIL  Output size: expected [%d × %d], got [%d × %d]\n', ...
        expectedRows, nDOF, size(qFull,1), size(qFull,2));
end

% -------------------------------------------------------------------------
% Test 2 — First frame matches first waypoint
% -------------------------------------------------------------------------
err = norm(qFull(1, :) - qWP(1, :));
ok  = err < 1e-10;
if ok, PASS = PASS+1; else, FAIL = FAIL+1; end
fprintf('  %s  First frame matches WP1  (err=%.2e)\n', ...
    iif(ok,'PASS','FAIL'), err);

% -------------------------------------------------------------------------
% Test 3 — Last frame of each segment matches the next waypoint
% -------------------------------------------------------------------------
allMatch = true;
for seg = 1:numSegs
    lastRow = seg * framesPerSeg;
    e = norm(qFull(lastRow, :) - qWP(seg+1, :));
    if e > 1e-10
        allMatch = false;
        fprintf('  FAIL  Segment %d endpoint error = %.2e\n', seg, e);
    end
end
ok = allMatch;
if ok, PASS = PASS+1; else, FAIL = FAIL+1; end
fprintf('  %s  All segment endpoints match waypoints\n', iif(ok,'PASS','FAIL'));

% -------------------------------------------------------------------------
% Test 4 — Velocity output dimensions
% -------------------------------------------------------------------------
ok = isequal(size(qdFull), size(qFull));
if ok, PASS = PASS+1; else, FAIL = FAIL+1; end
fprintf('  %s  Velocity output size matches position\n', iif(ok,'PASS','FAIL'));

% -------------------------------------------------------------------------
% Test 5 — Acceleration output dimensions
% -------------------------------------------------------------------------
ok = isequal(size(qddFull), size(qFull));
if ok, PASS = PASS+1; else, FAIL = FAIL+1; end
fprintf('  %s  Acceleration output size matches position\n', iif(ok,'PASS','FAIL'));

% -------------------------------------------------------------------------
% Test 6 — No large discontinuities between consecutive frames
% -------------------------------------------------------------------------
maxJump  = max(abs(diff(qFull)), [], 'all');
ok       = maxJump < 0.5;   % radians — generous bound for unit test
if ok, PASS = PASS+1; else, FAIL = FAIL+1; end
fprintf('  %s  No large frame-to-frame jumps  (max=%.4f rad)\n', ...
    iif(ok,'PASS','FAIL'), maxJump);

% -------------------------------------------------------------------------
% Summary
% -------------------------------------------------------------------------
fprintf('\n  Results: %d passed, %d failed\n\n', PASS, FAIL);


% ---- local helper -------------------------------------------------------
function s = iif(cond, a, b)
    if cond, s = a; else, s = b; end
end
