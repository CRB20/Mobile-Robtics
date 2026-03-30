clc; clear; close all;

addpath("C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\matlab")

client = RemoteAPIClient();
sim = client.require('sim');
sim.setStepping(true);

lidar = sim.getObject('/LiDAR/laser');
lidar_joint = sim.getObject('/LiDAR/joint');
leftMotor  = sim.getObject('/PioneerP3DX/leftMotor');
rightMotor = sim.getObject('/PioneerP3DX/rightMotor');




% -------------------------------------------------
% LiDAR parameters
% -------------------------------------------------
scanningAngle = 360 * pi/180;     % total FOV
stepSize      = 2*pi/1024;
maxScanDist   = 6;
pts           = 684;

angles = linspace(-scanningAngle/2, scanningAngle/2, pts);
ranges = maxScanDist * ones(1, pts);

% -------------------------------------------------
% Create figure (persistent)
% -------------------------------------------------
figure('Name','Real-time LiDAR Scan');
h = polarplot(angles, ranges, '.');
rlim([0 maxScanDist]);
title('LiDAR Streaming from CoppeliaSim');
grid on;

% Move robot
sim.startSimulation();
pause(1);

sim.setJointTargetVelocity(leftMotor, 1);
sim.setJointTargetVelocity(rightMotor, 1);
% -------------------------------------------------
% Continuous streaming loop
% -------------------------------------------------
while ishandle(h)

    for i = 1:pts
        % Rotate LiDAR joint
        sim.setJointPosition(lidar_joint, angles(i));
        

        % Read proximity sensor
        [res, dist, ~, ~] = sim.handleProximitySensor(lidar);

        if res > 0
            ranges(i) = dist;
        else
            ranges(i) = maxScanDist;
        end
        % Update plot
        set(h, 'RData', ranges);
        drawnow limitrate;
        %disp(min(ranges))
        sim.step();
    end

    
end

% -------------------------------------------------
% Stop simulation safely
% -------------------------------------------------
sim.stopSimulation();