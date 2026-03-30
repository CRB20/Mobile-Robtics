clc; clear; close all;
disp('INITIALIZING SUMMIT-XL 4-WHEEL EKF...');

% ================= CONNECT TO COPPELIASIM =================
addpath("C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\matlab")
client = RemoteAPIClient();
sim = client.require('sim');
sim.setStepping(true);
sim.startSimulation();

% ================= GET OBJECT HANDLES =================
robot      = sim.getObject('/summit_xl');
wheelFL    = sim.getObject('/summit_xl_front_left_wheel');
wheelFR    = sim.getObject('/summit_xl_front_right_wheel');
wheelBL    = sim.getObject('/back_left_wheel');
wheelBR    = sim.getObject('/back_right_wheel');
lidarJoint = sim.getObject('/summit_xl/LiDAR/joint');
lidarRay   = sim.getObject('/summit_xl/LiDAR/laser');

% ================= EKF INITIALIZATION & TUNING =================
startPos = sim.getObjectPosition(robot, -1);
xEst = [startPos{1}; startPos{2}; 0]; 
PEst = eye(3) * 0.01;                 
Q = diag([0.001, 0.001, 0.0005]); 
R = diag([0.01, 0.01]);       

% Robot Parameters
r = 0.165; L = 0.582; dt = 0.05;
v_target = 0.2;

% Initialize sensor memory
prevL = sim.getJointPosition(wheelFL);
prevR = sim.getJointPosition(wheelFR);

% Start 4-Wheel Motion
w_wheel = v_target / r;
sim.setJointTargetVelocity(wheelFL,  w_wheel+0.1);
sim.setJointTargetVelocity(wheelFR, -w_wheel); % Inverted for forward motion
sim.setJointTargetVelocity(wheelBL,  w_wheel+0.1);
sim.setJointTargetVelocity(wheelBR, -w_wheel); % Inverted for forward motion

% ================= PLOT SETUP =================
figure('Name', 'Summit-XL High Precision EKF', 'Color', 'w', 'Position', [100, 100, 1100, 500]);
subplot(1,2,1); h_scan = scatter(0, 0, 15, 'b', 'filled'); hold on; grid on; axis equal;
axis([-5 5 -5 5]); title('360° LiDAR Scan (Robot Frame)');
subplot(1,2,2); h_gt = plot(0, 0, 'r-', 'LineWidth', 2); hold on; 
h_est = plot(0, 0, 'g--', 'LineWidth', 1.5); grid on; axis equal;
legend('Actual Ground Truth', 'EKF Estimate');

% ================= MAIN LOOP =================
traj_gt_x = []; traj_gt_y = [];
traj_ekf_x = []; traj_ekf_y = [];

for k = 1:1200
    sim.step();
    
    % --- 1. SENSOR DATA ACQUISITION ---
    [~, v_a_cell] = sim.getObjectVelocity(robot);
    v_ang = cell2mat(v_a_cell);
    currL = sim.getJointPosition(wheelFL);
    currR = sim.getJointPosition(wheelFR);
    
    % Calculate Displacement 
    dL = r * (currL - prevL);
    dR = r * (currR - prevR);
    
    % FIX: Since right wheels are inverted (-w_wheel), dR will be negative.
    % We subtract dR (which is dL - (-dR)) to get the actual forward distance.
    v_enc = ((dL - dR) / (2 * dt)) + randn()*0.005; 
    gyroZ = v_ang(3) + randn()*0.002; 
    
    prevL = currL; prevR = currR;

    % --- 2. EKF PREDICTION ---
    theta_old = xEst(3);
    xPred = xEst + [v_enc*cos(theta_old)*dt; v_enc*sin(theta_old)*dt; gyroZ*dt];
    Fx = [1, 0, -v_enc*sin(theta_old)*dt; 0, 1, v_enc*cos(theta_old)*dt; 0, 0, 1];
    PPred = Fx * PEst * Fx' + Q;

    % --- 3. LIDAR SCANNING ---
    scan_points = [];
    num_steps = 24; 
    for i = 1:num_steps
        angle = (i-1) * (2*pi/num_steps);
        sim.setJointPosition(lidarJoint, angle);
        [res, dist, ~, ~, ~] = sim.checkProximitySensor(lidarRay, sim.handle_all);
        if res > 0
            scan_points = [scan_points; dist*cos(angle), dist*sin(angle)];
        end
    end
    
    % --- 4. EKF UPDATE ---
    pos_gt_cell = sim.getObjectPosition(robot, -1);
    pos_gt = [pos_gt_cell{1}; pos_gt_cell{2}];
    z = pos_gt + randn(2,1)*0.01; 
    
    H = [1, 0, 0; 0, 1, 0];
    y = z - H*xPred;
    S = H * PPred * H' + R;
    K = PPred * H' / S;
    
    xEst = xPred + K * y;
    PEst = (eye(3) - K * H) * PPred;

    % --- 5. VISUALIZATION ---
    traj_gt_x = [traj_gt_x; pos_gt(1)]; traj_gt_y = [traj_gt_y; pos_gt(2)];
    traj_ekf_x = [traj_ekf_x; xEst(1)]; traj_ekf_y = [traj_ekf_y; xEst(2)];
    
    if ~isempty(scan_points)
        set(h_scan, 'XData', scan_points(:,1), 'YData', scan_points(:,2));
    end
    set(h_gt, 'XData', traj_gt_x, 'YData', traj_gt_y);
    set(h_est, 'XData', traj_ekf_x, 'YData', traj_ekf_y);
    title(subplot(1,2,2), sprintf('Step %d | Error: %.4f m', k, norm(pos_gt - xEst(1:2))));
    drawnow limitrate;
end

% ================= STOP SIMULATION =================
sim.setJointTargetVelocity(wheelFL, 0);
sim.setJointTargetVelocity(wheelFR, 0);
sim.setJointTargetVelocity(wheelBL, 0);
sim.setJointTargetVelocity(wheelBR, 0);
sim.stopSimulation();
disp('SIMULATION FINISHED.');