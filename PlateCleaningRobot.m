clc;
clear;
close all;

% {'analytic','numeric','generalized'}
ik_method = "numeric";

% Load ABB IRB 120
r = loadrobot('abbIrb120', 'DataFormat', 'column');
q0 = zeros(6,1);
q = q0;

% Task space boundaries
x_min = 0.0; x_max = 0.3;
y_min = -0.38; y_max = 0.38;
z_plate = 0.03;  % Plate height
minDistance = 0.28;  % Minimum distance between plates
baseRadius = 0.25;   % Base exclusion zone

% Number of plates
numPlates = 4;

% Random plate position generator
generateRandomPosition = @() [x_min + (x_max-x_min)*rand, y_min + (y_max-y_min)*rand, z_plate];

% Generate positions avoiding overlap & base
platePositions = zeros(numPlates, 3);
for i = 1:numPlates
    isValid = false;
    while ~isValid
        newPosition = generateRandomPosition();
        distances = sqrt(sum((platePositions(1:i-1,1:2) - newPosition(1:2)).^2, 2));
        distanceFromBase = norm(newPosition(1:2));
        if all(distances > minDistance) && distanceFromBase > baseRadius
            platePositions(i, :) = newPosition;
            isValid = true;
        end
    end
end

% Initialize plot
figure;
ax = axes;
show(r, q, 'Parent', ax, 'Visuals', 'on');
hold(ax, 'on');
drawFloor();

% Add plates
for idx = 1:numPlates
    PlatePosition = platePositions(idx, :);
    body = rigidBody(['Plate', num2str(idx), '_link']);
    addVisual(body, "Cylinder", [0.14, 0.01]);  % [radius, length]
    setFixedTransform(body.Joint, trvec2tform(PlatePosition));
    addBody(r, body, r.BaseName);
    show(r, 'Parent', ax, 'Visuals', 'on', 'PreservePlot', 0, 'Frames', 'off');
    drawnow;
end

% IK
ik = inverseKinematics('RigidBodyTree', r);
orientation = eul2quat([0, pi/2, 0]);  % z-axis down

% PD control params
Kp = 20;
Kd = 0.1;
dt = 0.01;

visitedPlates = false(numPlates, 1);
currentPosition = [0, 0, 0];

for visitCount = 1:numPlates
    distances = sqrt(sum((platePositions(:,1:2) - currentPosition(1:2)).^2, 2));
    distances(visitedPlates) = inf;
    [~, idx] = min(distances);
    PlatePosition = platePositions(idx, :);
    visitedPlates(idx) = true;

    % Spiral Trajectory
    nPoints = 55;
    radius = 0.145;
    turns = 3;
    theta = linspace(0, 2*pi*turns, nPoints);
    radii = linspace(0, radius, nPoints);
    z_above_plate = 0.05;
    z = ones(1, nPoints) * (z_plate + z_above_plate);
    x = radii .* cos(theta) + PlatePosition(1);
    y = radii .* sin(theta) + PlatePosition(2);

    prevError = zeros(6,1);
    for i = 1:nPoints
        Td = trvec2tform([x(i), y(i), z(i)]) * quat2tform(orientation);
        [q_desired, ~] = ik('tool0', Td, ones(6,1), q);
        error = q_desired - q;
        dError = (error - prevError) / dt;
        u = Kp * error + Kd * dError;
        q = q + u * dt;
        prevError = error;

        show(r, q, 'Parent', ax, 'Visuals', 'on', 'PreservePlot', 0, 'Frames', 'off');
        plotTransforms(Td(1:3,4)', tform2quat(Td), 'Parent', ax, 'framesize', 0.05);
        plot3(ax, x(i), y(i), z(i), 'mo', 'MarkerSize', 4, 'MarkerFaceColor', 'm');
        drawnow;
    end

    currentPosition = [x(end), y(end), z(end)];

    % Transition Trajectory
    if visitCount < numPlates
        distances = sqrt(sum((platePositions(:,1:2) - currentPosition(1:2)).^2, 2));
        distances(visitedPlates) = inf;
        [~, nextIdx] = min(distances);
        nextPlatePosition = platePositions(nextIdx, :);

        nTransitionPoints = 50;
        transitionTime = linspace(0, 1, nTransitionPoints);

        angle1 = atan2(currentPosition(2), currentPosition(1));
        angle2 = atan2(nextPlatePosition(2), nextPlatePosition(1));
        if angle1 < 0, angle1 = angle1 + 2*pi; end
        if angle2 < 0, angle2 = angle2 + 2*pi; end
        if angle2 < angle1, angle2 = angle2 + 2*pi; end

        clockwiseDistance = angle2 - angle1;
        counterClockwiseDistance = 2*pi - clockwiseDistance;

        if clockwiseDistance <= counterClockwiseDistance
            midAngles = linspace(angle1, angle2, 5);
        else
            midAngles = linspace(angle1, angle2 - 2*pi, 5);
        end

        midRadius = max(baseRadius + 0.1, 0.3);
        midPoints = [midRadius*cos(midAngles') midRadius*sin(midAngles') repmat(z_above_plate + 0.2, length(midAngles), 1)];
        waypoints = [currentPosition; midPoints; nextPlatePosition(1), nextPlatePosition(2), z_plate + z_above_plate];
        waypointsTime = linspace(0, 1, size(waypoints, 1));

        [transitionTraj, ~, ~] = cubicpolytraj(waypoints', waypointsTime, transitionTime);
        xTransition = transitionTraj(1, :);
        yTransition = transitionTraj(2, :);
        zTransition = transitionTraj(3, :);

        prevError = zeros(6,1);
        for i = 1:nTransitionPoints
            Td = trvec2tform([xTransition(i), yTransition(i), zTransition(i)]) * quat2tform(orientation);
            [q_desired, ~] = ik('tool0', Td, ones(6,1), q);
            error = q_desired - q;
            dError = (error - prevError) / dt;
            u = Kp * error + Kd * dError;
            q = q + u * dt;
            prevError = error;

            show(r, q, 'Parent', ax, 'Visuals', 'on', 'PreservePlot', 0, 'Frames', 'off');
            plotTransforms(Td(1:3,4)', tform2quat(Td), 'Parent', ax, 'framesize', 0.05);
            drawnow;
        end

        currentPosition = [xTransition(end), yTransition(end), zTransition(end)];
    end
end

% Floor helper
function drawFloor()
    ax = gca;
    ax.CameraViewAngle = 5;
    p = patch([-1 1 1 -1]*0.5, [-1 -1 1 1]*0.5, [0 0 0 0]);
    p.FaceColor = [0.8, 0.8, 0.8];
    axis off;
    xlim([-0.75, 0.75]);
    ylim([-0.75, 0.75]);
    zlim([0,0.75]);
end