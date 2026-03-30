clc; clear; close all;

% -------------------------------------------------
% Add ZMQ Remote API path
% -------------------------------------------------
addpath("C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\matlab")

% -------------------------------------------------
% Connect to CoppeliaSim
% -------------------------------------------------
client = RemoteAPIClient();
sim = client.require('sim');

sim.startSimulation();
pause(1);

% -------------------------------------------------
% Get handles
% -------------------------------------------------
radar      = sim.getObject('/Radar');
robot      = sim.getObject('/PioneerP3DX');
leftMotor  = sim.getObject('/PioneerP3DX/leftMotor');
rightMotor = sim.getObject('/PioneerP3DX/rightMotor');

% -------------------------------------------------
% Move robot forward
% -------------------------------------------------
sim.setJointTargetVelocity(leftMotor,  2);
sim.setJointTargetVelocity(rightMotor, 2);

% -------------------------------------------------
% Radar parameters
% -------------------------------------------------
numBeams = 120;
maxRange = 10;
angles   = linspace(-pi/3, pi/3, numBeams);

ranges = maxRange * ones(1, numBeams);

% -------------------------------------------------
% Radar scan loop
% -------------------------------------------------
for i = 1:100

    for k = 1:numBeams

        % Rotate radar
        
        sim.setObjectOrientation(radar, robot, [0 deg2rad(90) deg2rad(angles(k))]);

        % Read proximity sensor
        [res, dist, ~, ~] = sim.readProximitySensor(radar);

        if res > 0
            ranges(k) = dist;
             fprintf('Object detected at distance %.2f m, angle %.2f deg\n', ...
            dist, rad2deg(angles(k)));
        else
            ranges(k) = maxRange;
        end
    end

    % Convert polar to Cartesian
    x = ranges .* cos(angles);
    y = ranges .* sin(angles);

    % Plot radar scan
    clf
    scatter(x, y, 20, 'filled')
    axis equal
    grid on
    xlim([-maxRange maxRange])
    ylim([-maxRange maxRange])
    title('RADAR Scan (2D)')
    xlabel('X (m)')
    ylabel('Y (m)')
    drawnow

    pause(0.1);
end

% -------------------------------------------------
% Stop robot & simulation
% -------------------------------------------------
sim.setJointTargetVelocity(leftMotor,  0);
sim.setJointTargetVelocity(rightMotor, 0);
sim.stopSimulation();




