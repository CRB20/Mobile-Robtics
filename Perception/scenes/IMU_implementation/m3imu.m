clc; clear; close all;

addpath("C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\matlab")

client = RemoteAPIClient();
sim = client.require('sim');

sim.startSimulation();
pause(1);

% -------------------------
% Get handles
% -------------------------
imu        = sim.getObject('/IMU');
leftMotor  = sim.getObject('/PioneerP3DX/leftMotor');
rightMotor = sim.getObject('/PioneerP3DX/rightMotor');

% -------------------------
% Move robot forward
% -------------------------
sim.setJointTargetVelocity(leftMotor,  2);
sim.setJointTargetVelocity(rightMotor, 2);

dt = 0.05;
prevVel = [0 0 0];

% -------------------------
% IMU loop
% -------------------------
for i = 1:200

    [linearVel, angularVel] = sim.getObjectVelocity(imu);
    if isempty(linearVel)
        continue;
    end

    linearVel  = linearVel{1};
    angularVel = angularVel{1};

    accel = (linearVel - prevVel) / dt;
    prevVel = linearVel;

    omega = angularVel;

    euler = sim.getObjectOrientation(imu, -1);
    euler = euler{1};

    fprintf('Sample %d\n', i);
    fprintf('  Accel [m/s^2] : X = %.2f | Y = %.2f | Z = %.2f\n', accel(1), accel(2), accel(3));
    fprintf('  Gyro  [rad/s] : X = %.2f | Y = %.2f | Z = %.2f\n', omega(1), omega(2), omega(3));
    fprintf('  Euler [rad]   : Roll = %.2f | Pitch = %.2f | Yaw = %.2f\n', euler(1), euler(2), euler(3));
    fprintf('---------------------------------------------\n');

    pause(dt);
end
