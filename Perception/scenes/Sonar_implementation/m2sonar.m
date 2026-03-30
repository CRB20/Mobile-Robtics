clc; clear; close all;

addpath("C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\matlab")
client = RemoteAPIClient();
sim = client.require('sim');

sim.startSimulation();
pause(1);

% Get ultrasonic sensor handle
ultraSensor = sim.getObject('/ultrasonic_front');

for i = 1:200
    % Read the proximity sensor
    [result, distance, detectedPoint, detectedObjectHandle, surfaceNormalVector] = ...
        sim.readProximitySensor(ultraSensor);
    
    if result > 0
        %dist = norm(detectedPoint);   % distance to object
        %fprintf("Distance: %.2f cm\n", dist*100);
        fprintf("Object detected\n");
    else
        fprintf("No object detected\n");
    end

    pause(0.05);
end

sim.stopSimulation();