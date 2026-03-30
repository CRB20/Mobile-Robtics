clc; clear; close all;
disp('INITIALIZING HIGH-DENSITY SLAM MAPPER...');

% ================= CONNECT TO COPPELIASIM =================
addpath("C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\matlab")
client = RemoteAPIClient();
sim = client.require('sim');
sim.setStepping(true);
sim.startSimulation();

% ================= GET OBJECT HANDLES =================
robot = sim.getObject('/summit_xl');
wheels = [sim.getObject('/summit_xl_front_left_wheel'), ...
          sim.getObject('/summit_xl_front_right_wheel'), ...
          sim.getObject('/back_left_wheel'), ...
          sim.getObject('/back_right_wheel')];
lidarJoint = sim.getObject('/summit_xl/LiDAR/joint');
lidarRay   = sim.getObject('/summit_xl/LiDAR/laser');

% ================= MAP & POSITION SETUP =================
startPos = sim.getObjectPosition(robot, -1);
xEst = [startPos{1}; startPos{2}; 0]; 
dt = 0.05; r = 0.165; L = 0.582;

% Map Parameters
mapSize = 20; res = 0.1; gridN = round(mapSize/res);
% Initialize map with 0.5 (Unknown gray)
occMap = 0.5 * ones(gridN, gridN); 

% ================= STABLE UI SETUP =================
fig = figure('Name', 'Summit-XL Mapper', 'NumberTitle', 'off');
userData.currentKey = 'none';
set(fig, 'UserData', userData);
set(fig, 'KeyPressFcn', @(src, event) updateKey(src, event, true));
set(fig, 'KeyReleaseFcn', @(src, event) updateKey(src, event, false));

ax = subplot(1,1,1);
h_map = imagesc([-mapSize/2 mapSize/2], [-mapSize/2 mapSize/2], occMap);
colormap(gray); colorbar; set(gca, 'YDir', 'normal'); hold on;
h_path = plot(xEst(1), xEst(2), 'y-', 'LineWidth', 1.5); % Path history
h_rob = plot(xEst(1), xEst(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
axis equal; title('Driving: WASD | Mapping: Active (Ray-Casting)');

% ================= MAIN LOOP =================
traj_x = xEst(1); traj_y = xEst(2);

while ishandle(fig)
    uData = get(fig, 'UserData');
    currentKey = uData.currentKey;
    if strcmp(currentKey, 'q'), break; end
    
    % --- 1. CONTROL LOGIC ---
    v = 0; w = 0;
    if strcmp(currentKey, 'w'), v = 0.5; end
    if strcmp(currentKey, 's'), v = -0.5; end
    if strcmp(currentKey, 'a'), w = 0.8; end
    if strcmp(currentKey, 'd'), w = -0.8; end
    
    wl = (v - w*L/2)/r; wr = (v + w*L/2)/r;
    sim.setJointTargetVelocity(wheels(1), wl); sim.setJointTargetVelocity(wheels(3), wl);
    sim.setJointTargetVelocity(wheels(2), -wr); sim.setJointTargetVelocity(wheels(4), -wr);
    
    sim.step();
    
    % --- 2. ODOMETRY ---
    theta = xEst(3);
    xEst = xEst + [v*cos(theta)*dt; v*sin(theta)*dt; w*dt];
    traj_x = [traj_x; xEst(1)]; traj_y = [traj_y; xEst(2)];
    
    % --- 3. DENSE MAPPING (RAY CASTING) ---
    num_steps = 90; % Angles per cycle
    for i = 1:num_steps
        angle = (i-1) * (2*pi/num_steps);
        sim.setJointPosition(lidarJoint, angle);
        [res_lidar, dist, ~, ~, ~] = sim.checkProximitySensor(lidarRay, sim.handle_all);
        
        % Robot Position in Grid
        x0 = round((xEst(1) + mapSize/2) / res);
        y0 = round((xEst(2) + mapSize/2) / res);
        
        if res_lidar > 0 && dist < 6.0
            % Hit Position in Grid
            gx = xEst(1) + dist * cos(xEst(3) + angle);
            gy = xEst(2) + dist * sin(xEst(3) + angle);
            x1 = round((gx + mapSize/2) / res);
            y1 = round((gy + mapSize/2) / res);
            
            % RAY CASTING (Bresenham's)
            [pixelsX, pixelsY] = bresenham(x0, y0, x1, y1);
            
            for p = 1:length(pixelsX)
                px = pixelsX(p); py = pixelsY(p);
                if px > 0 && px <= gridN && py > 0 && py <= gridN
                    if p == length(pixelsX)
                        % End of ray = Obstacle (Black)
                        occMap(py, px) = max(occMap(py, px) - 0.1, 0); 
                    else
                        % Path of ray = Free Space (White)
                        occMap(py, px) = min(occMap(py, px) + 0.02, 1);
                    end
                end
            end
        end
    end
    
    % --- 4. RENDER ---
    if ishandle(h_map)
        set(h_map, 'CData', occMap);
        set(h_rob, 'XData', xEst(1), 'YData', xEst(2));
        set(h_path, 'XData', traj_x, 'YData', traj_y);
        drawnow limitrate;
    end
end

sim.stopSimulation();

% ================= HELPER FUNCTIONS =================
function updateKey(src, event, isPressed)
    uData = get(src, 'UserData');
    if isPressed, uData.currentKey = event.Key; else, uData.currentKey = 'none'; end
    set(src, 'UserData', uData);
end

function [x, y] = bresenham(x1, y1, x2, y2)
    % Simple Bresenham Line Algorithm
    dx = abs(x2 - x1); dy = abs(y2 - y1);
    steep = abs(dy) > abs(dx);
    if steep, [x1, y1] = deal(y1, x1); [x2, y2] = deal(y2, x2); end
    if x1 > x2, [x1, x2] = deal(x2, x1); [y1, y2] = deal(y2, y1); end
    dx = x2 - x1; dy = abs(y2 - y1);
    error = dx / 2;
    ystep = -1; if y1 < y2, ystep = 1; end
    y = zeros(1, x2-x1+1);
    currY = y1;
    for currX = x1:x2
        if steep, y(currX-x1+1) = currX; x(currX-x1+1) = currY;
        else, x(currX-x1+1) = currX; y(currX-x1+1) = currY; end
        error = error - dy;
        if error < 0, currY = currY + ystep; error = error + dx; end
    end
end
