clc; clear; close all;

% -----------------------------
% CONNECT TO COPPELIASIM
% -----------------------------
addpath("C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\matlab");
client = RemoteAPIClient();
sim = client.require('sim');

sim.startSimulation();
pause(1);

% -----------------------------
% GET HANDLES
% -----------------------------
robot      = sim.getObject('/PioneerP3DX');
leftMotor  = sim.getObject('/PioneerP3DX/leftMotor');
rightMotor = sim.getObject('/PioneerP3DX/rightMotor');

% -----------------------------
% PARAMETERS
% -----------------------------
ticks_per_revolution = 500;        % Encoder resolution
wheel_radius = 0.0975/2;           % Pioneer P3DX wheel radius (m)
wheel_base   = 0.33;               % Distance between wheels (m)

encoder_noise_std = 2;             % ±2 tick noise

% ODOMETRY INITIAL VALUES
x = 0;          
y = 0;
theta = 0;     

prev_left_ticks = 0;
prev_right_ticks = 0;

fprintf("Starting Encoder + Odometry Simulation...\n");

% -----------------------------
% MAIN LOOP
% -----------------------------
for t = 1:1000     % run for some steps
    
    %============================================================
    % 1) READ WHEEL JOINT POSITIONS (radians)
    %============================================================
    left_pos  = sim.getJointPosition(leftMotor);   
    right_pos = sim.getJointPosition(rightMotor);
    
    %============================================================
    % 2) RAD → TICKS
    %============================================================
    left_ticks  = round((left_pos  / (2*pi)) * ticks_per_revolution);
    right_ticks = round((right_pos / (2*pi)) * ticks_per_revolution);
    
    %============================================================
    % 3) ADD ENCODER NOISE
    %============================================================
    left_ticks_noisy  = left_ticks  + round(encoder_noise_std * randn());
    right_ticks_noisy = right_ticks + round(encoder_noise_std * randn());
    
    %============================================================
    % 4) DELTA TICKS
    %============================================================
    dleft  = left_ticks_noisy  - prev_left_ticks;
    dright = right_ticks_noisy - prev_right_ticks;
    
    prev_left_ticks  = left_ticks_noisy;
    prev_right_ticks = right_ticks_noisy;
    
    %============================================================
    % 5) TICKS → ANGLE (rad)
    %============================================================
    dphi_left  = (dleft  / ticks_per_revolution) * 2*pi;
    dphi_right = (dright / ticks_per_revolution) * 2*pi;
    
    %============================================================
    % 6) ANGLE → DISTANCE
    %============================================================
    dleft_dist  = dphi_left  * wheel_radius;
    dright_dist = dphi_right * wheel_radius;
    
    %============================================================
    % 7) ODOMETRY UPDATE
    %============================================================
    d = (dright_dist + dleft_dist) / 2;
    dtheta = (dright_dist - dleft_dist) / wheel_base;
    
    x = x + d * cos(theta + dtheta/2);
    y = y + d * sin(theta + dtheta/2);
    theta = theta + dtheta;
    
    % Normalize angle
    theta = mod(theta + pi, 2*pi) - pi;
    
    %============================================================
    % 8) PRINT ODOMETRY VALUES
    %============================================================
    fprintf("t=%d | x=%.3f m | y=%.3f m | theta=%.3f rad\n", t, x, y, theta);
    
    pause(0.05);
end

% -----------------------------
% STOP SIMULATION
% -----------------------------
sim.stopSimulation();
pause(1);

fprintf("\nSimulation finished.\n");
