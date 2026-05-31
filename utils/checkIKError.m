function posErr = checkIKError(robot, endEffector, qSol, targetXYZ, wpIndex, threshold)
%CHECKIKERROR  Report position error for a solved IK configuration.
%
%  Computes the Euclidean distance between the achieved end-effector position
%  (from forward kinematics on qSol) and the desired Cartesian position.
%  Prints a formatted status line to the console.
%
%  Inputs
%  ------
%    robot        : rigidBodyTree (DataFormat = "row")
%    endEffector  : name of the end-effector body (string)
%    qSol         : 1×N solved joint configuration
%    targetXYZ    : 1×3 desired [x, y, z] position in metres
%    wpIndex      : waypoint index (integer, for display only)
%    threshold    : position error threshold in metres
%                   (prints OK if err < threshold, POOR otherwise)
%
%  Output
%  ------
%    posErr : scalar Euclidean position error in metres
%
%  Example
%  -------
%    err = checkIKError(gen3, 'EndEffector_Link', qSol, [0.4 0.0 0.5], 1, 0.01);
%
%  See also: basicIK, main

T_achieved = getTransform(robot, qSol, endEffector);
posErr     = norm(tform2trvec(T_achieved) - targetXYZ);

if posErr < threshold
    status = 'OK  ';
else
    status = 'POOR';
end

fprintf('  Waypoint %2d:  %s   err = %.4f m\n', wpIndex, status, posErr);
end
