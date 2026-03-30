clc; clear; close all;

% -------------------------------------------------
% Add ZMQ Remote API path
% -------------------------------------------------
addpath("C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\matlab")
client = RemoteAPIClient();
sim = client.require('sim');
simVision = client.require('simVision');
sim.setStepping(true);


sim.startSimulation();
pause(1);

% -------------------------------------------------
% Get handles
% -------------------------------------------------
cam = sim.getObject('/PioneerP3DX/kinect/rgb');
cam_depth = sim.getObject('/PioneerP3DX/kinect/depth');
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
hFigRGB   = figure('Name','RGB Camera');
hImgRGB  = imshow(zeros(128,128,3,'uint8'));
title('RGB Image');

hFigDepth  = figure('Name','Depth Camera');
hImgDepth = imshow(zeros(128,128),'InitialMagnification','fit');
colormap('jet');
colorbar;
title('Depth Image (0 = Near, 1 = Far)');





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

    % -------------------------------------------------
    % Depth sensor
    % -------------------------------------------------
    depthCell_packed = sim.getVisionSensorDepth(cam_depth);
    depthCell = sim.unpackFloatTable(depthCell_packed);
    if ~isempty(depthCell)
        depth = cell2mat(depthCell);
        depth = reshape(depth, [imgwidth, imgheight]);
        depth = depth';               % match RGB orientation
        depth = flipud(depth);

        set(hImgDepth, 'CData', depth);
    end

    drawnow;


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
    set(hImgRGB, 'CData', img);
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
