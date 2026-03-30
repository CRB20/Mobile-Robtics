clc; clear; close all;

% -------------------------------------------------
% Add ZMQ Remote API path
% -------------------------------------------------
addpath("C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\matlab")
client = RemoteAPIClient();
sim = client.require('sim');

sim.setStepping(true);


sim.startSimulation();
pause(1);

% -------------------------------------------------
% Get handles
% -------------------------------------------------
cam        = sim.getObject('/PioneerP3DX/DVS128/sensor');
leftMotor  = sim.getObject('/PioneerP3DX/leftMotor');
rightMotor = sim.getObject('/PioneerP3DX/rightMotor');

% -------------------------------------------------
% Move robot forward
% -------------------------------------------------
sim.setJointTargetVelocity(leftMotor,  2);
sim.setJointTargetVelocity(rightMotor, 2);


% -------------------------------------------------
% Create figure and image object for live streaming
% -------------------------------------------------
hFig = figure('Name','Camera Output from CoppeliaSim');
hImg = imshow(zeros(128, 128, 3, 'uint8')); % initial black frame
title('Camera Output from CoppeliaSim');


% -------------------------------------------------
% Read camera image loop
% -------------------------------------------------
for i = 1:200

    % Get image (ZMQ returns CELL)
    img = sim.getVisionSensorImage(cam);
    resolution = sim.getVisionSensorRes(cam);
    % -------------------------------------------------
    % Camera parameters (must match CoppeliaSim)
    % -------------------------------------------------
    resolution= cell2mat(resolution);
    imgwidth  = resolution(1);
    imgheight = resolution(2);


    if isempty(img)
        continue;
    end

    % Extract numeric data from cell
    %img = img{1};
    img = cell2mat(img);
    % Convert to MATLAB image
    img = reshape(img, [3, imgwidth, imgheight]);
    img = permute(img, [3 2 1]);        % H × W × 3
    img = flipud(img);                  % Fix orientation
    img = uint8((img + 1) * 127.5);     % [-1,1] → [0,255]

    % Display image
    set(hImg, 'CData', img);
    drawnow

    %pause(0.05);
    sim.step();
end

% -------------------------------------------------
% Stop robot & simulation
% -------------------------------------------------
sim.setJointTargetVelocity(leftMotor,  0);
sim.setJointTargetVelocity(rightMotor, 0);

sim.stopSimulation();
