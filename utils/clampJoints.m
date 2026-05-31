function q = clampJoints(robot, q)
%CLAMPJOINTS  Clamp a joint configuration to each joint's position limits.
%
%  Reads PositionLimits from each body's joint in the rigidBodyTree and
%  saturates the corresponding element of q.
%
%  Inputs
%  ------
%    robot : rigidBodyTree (DataFormat = "row")
%    q     : 1×N joint configuration (row vector, radians)
%
%  Output
%  ------
%    q     : 1×N clamped joint configuration
%
%  Example
%  -------
%    gen3 = loadrobot("kinovaGen3", DataFormat="row");
%    q    = clampJoints(gen3, rand(1,7) * 2*pi - pi);
%
%  See also: basicIK

nJoints = numel(q);

for j = 1:nJoints
    lims = robot.Bodies{j}.Joint.PositionLimits;
    q(j) = max(lims(1), min(lims(2), q(j)));
end
end
