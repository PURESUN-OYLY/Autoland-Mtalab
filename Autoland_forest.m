
clear; clc; close all; % Clear the workspace

%% Public params
Slope = 5;          % Maximum slope (5°)
lidar_hFOV = 120;   % Horizontal field of lidar view (120°)
lidar_vFOV = 60;    % Vertical field of lidar view (60°)

%% Forest Model
% 1. Environment parameters
fieldSize = 100;    % Map size (m)
gridRes = 0.5;      % Grid resolution (m), more grid points for higher resolution, but will be slower
[X, Y] = meshgrid(0:gridRes:fieldSize);

% 2. Generate ground
Z_ground = 4 * sin(X/15) .* cos(Y/15) + 2 * cos(X/25) + 0.3 * randn(size(X))*0.05;
Z_ground = Z_ground - min(Z_ground,[],'all');

% 3. Slope analysis and valid point search (slope < 5°)
[dzdx, dzdy] = gradient(Z_ground, gridRes);
slopeRad = atan(sqrt(dzdx.^2 + dzdy.^2));   % Calculate slope in radians
slopeDeg = rad2deg(slopeRad);               % Convert to degrees for visualization
validSlopeMask = slopeDeg < 5;          % Find valid slopes, which are less than Slope

% 4. Render environment
figure('Color', 'w', 'Name', 'Drone Forest Simulation');
set(gcf, 'Position', [100, 100, 1200, 800]);
s = surf(X, Y, Z_ground, 'EdgeColor', 'none', 'FaceAlpha', 0.8);
colormap(summer); hold on; light; lighting gouraud;

% 5. Plant trees in the forest (while updating the mask, trees below cannot land)
treeDensity = 0.015;
numTrees = round(fieldSize^2 * treeDensity);
rng(20);
treeLocations = []; % Record tree centers [x, y, r, h] for radar collision detection
for i = 1:numTrees
    tx = rand*fieldSize; ty = rand*fieldSize;
    if norm([tx, ty] - [5, 5]) > 5
        h_base = interp2(X, Y, Z_ground, tx, ty);
        trunkR = 0.6 + rand*0.3;
        trunkH = 5 + rand*3;

        % Record tree parameters for radar collision detection
        treeLocations = [treeLocations; tx, ty, trunkR, trunkH, h_base];

        % Render tree in the forest
        [cX, cY, cZ] = cylinder([1, 1], 12);
        surf(cX*trunkR + tx, cY*trunkR + ty, cZ*trunkH + h_base, 'FaceColor', [0.45, 0.25, 0.05], 'EdgeColor', 'none');
        [sx, sy, sz] = sphere(16); canopyR = 2.5 + rand*1.5;
        surf(sx*canopyR + tx, sy*canopyR + ty, sz*canopyR + h_base + trunkH, 'FaceColor', [0.1, 0.4, 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

        % Mark ground below trees as not landable (consider trunk and canopy projection)
        treeDist = sqrt((X - tx).^2 + (Y - ty).^2);
        validSlopeMask(treeDist < (trunkR + 1.5)) = 0;  % Add 1.5m safe distance
    end
end

% 6. Find connected regions with area >= 1x1m^2 (based on connected components)
% The area must be >= 1x1m^2 to be considered as a valid landing zone
minPixels = ceil(1.0 / gridRes^2);
connectedRegions = bwpropfilt(validSlopeMask, 'Area', [minPixels, inf]);

% 7. Mark valid landing zones in the forest
[r, c] = find(connectedRegions);
if ~isempty(r)
    % Use yellow points to mark valid landing zones in the forest
    scatter3(X(connectedRegions), Y(connectedRegions), Z_ground(connectedRegions)+0.1, ...
        20, 'y', 'filled', 'MarkerEdgeAlpha', 0.3, 'MarkerFaceAlpha', 0.3);

    % Automatically extract the centermost valid landing zone as the target landing position
    targetIdx = round(length(r)/2);
    targetX = X(r(targetIdx), c(targetIdx));
    targetY = Y(r(targetIdx), c(targetIdx));
    targetH = Z_ground(r(targetIdx), c(targetIdx));
    targetPos = [targetX; targetY; targetH + 3];    % Set target position to 3m above the centermost valid landing zone [cite: 25]
else
    targetPos = [80; 80; 13]; % Backup default point
end

% 8. Mark start point
plot3(5, 5, interp2(X, Y, Z_ground, 5, 5)+1, 'bp', 'MarkerSize', 15, 'MarkerFaceColor', 'b');
text(5, 5, interp2(X, Y, Z_ground, 5, 5)+4, 'Start point', 'FontWeight', 'bold');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Height (m)');
view(-45, 35); axis tight;

%% ---2. Drone Model ---
% 1. Initialize drone (call Autoland_drone.m)
uav = Autoland_drone([5; 5; 10]);

% 3D Drone Visualization
% 2. Drone physical dimensions
armLength = 1.2;  % Arm length (m)
propRadius = 0.4; % Propeller radius (m)

% 3. Drone arms
h_arm1 = plot3([0,0], [0,0], [0,0], 'Color', [0.2 0.2 0.2], 'LineWidth', 3);
h_arm2 = plot3([0,0], [0,0], [0,0], 'Color', [0.2 0.2 0.2], 'LineWidth', 3);

% 4. Propellers
theta_circle = linspace(0, 2*pi, 12);
cx = propRadius * cos(theta_circle);
cy = propRadius * sin(theta_circle);
cz = zeros(size(theta_circle));
h_prop1 = fill3(cx, cy, cz, 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); % Front-left (green)
h_prop2 = fill3(cx, cy, cz, 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); % Front-right (green)
h_prop3 = fill3(cx, cy, cz, 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); % Back-left (red)
h_prop4 = fill3(cx, cy, cz, 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); % Back-right (red)

% 5. Drone path
h_path = plot3(uav.Position(1), uav.Position(2), uav.Position(3), 'b-', 'LineWidth', 1.5);
uav_history = uav.Position;

%% ---3. Radar Beam Visualization ---
% Radar FOV, 120°x60°
[h_angles, v_angles] = meshgrid(linspace(-lidar_hFOV/2, lidar_hFOV/2, 5), linspace(-lidar_vFOV/2, lidar_vFOV/2, 3));
h_rays = cell(numel(h_angles), 1);
for k = 1:numel(h_angles)
    h_rays{k} = plot3([0,0], [0,0], [0,0], 'y:', 'LineWidth', 1); % Yellow dashed line represents radar beam
end

%% ---4. Simulation time parameters ---
totalTime = 100;
% dt = uav.dt;
steps = totalTime / uav.dt;

%% ---5. Simulation main loop ---
axis equal; % Maintain equal aspect ratio for 3D visualization
for t = 1:steps
    % --- Algorithm layer ---
    currentStepGoal = targetPos;

    % --- Physical layer layer ---
    uav.update(currentStepGoal);

    % --- Position layer layer ---
    pos = uav.Position;
    yaw = uav.Yaw;

    % Rotation Matrix by z-axis
    R = [cos(yaw), -sin(yaw), 0;
        sin(yaw),  cos(yaw), 0;
        0,         0,        1];

    % Calculate four arm endpoints positions
    p1 = R * [ armLength;  armLength; 0] + pos; % Front-left
    p2 = R * [-armLength; -armLength; 0] + pos; % Back-right
    p3 = R * [ armLength; -armLength; 0] + pos; % Front-right
    p4 = R * [-armLength;  armLength; 0] + pos; % Back-left

    % --- Visual layer (update at 30fps) ---
    if mod(t, 2) == 0
        % 1. Update arm positions
        set(h_arm1, 'XData', [p1(1), p2(1)], 'YData', [p1(2), p2(2)], 'ZData', [p1(3), p2(3)]);
        set(h_arm2, 'XData', [p3(1), p4(1)], 'YData', [p3(2), p4(2)], 'ZData', [p3(3), p4(3)]);

        % 2. Update 4 propeller positions
        set(h_prop1, 'XData', cx + p1(1), 'YData', cy + p1(2), 'ZData', cz + p1(3));
        set(h_prop2, 'XData', cx + p3(1), 'YData', cy + p3(2), 'ZData', cz + p3(3));
        set(h_prop3, 'XData', cx + p4(1), 'YData', cy + p4(2), 'ZData', cz + p4(3));
        set(h_prop4, 'XData', cx + p2(1), 'YData', cy + p2(2), 'ZData', cz + p2(3));

        % 3. Update dynamic radar scan rays (120x60°)
        ray_idx = 1;
        for r_h = linspace(-lidar_hFOV/2, lidar_hFOV/2, 5)
            for r_v = linspace(-lidar_vFOV/2, lidar_vFOV/2, 3)
                % Calculate ray direction
                total_yaw = yaw + deg2rad(r_h);
                dir = [cos(total_yaw)*cosd(r_v); sin(total_yaw)*cosd(r_v); sind(r_v)];
                ray_end = pos + dir * 15; % Radar view distance 15m

                set(h_rays{ray_idx}, 'XData', [pos(1), ray_end(1)], ...
                    'YData', [pos(2), ray_end(2)], ...
                    'ZData', [pos(3), ray_end(3)]);
                ray_idx = ray_idx + 1;
            end
        end

        % 4. Update trajectory line
        uav_history = [uav_history, pos];
        set(h_path, 'XData', uav_history(1,:), 'YData', uav_history(2,:), 'ZData', uav_history(3,:));

        drawnow limitrate;
    end

    % Arrival check
    if norm(uav.Position - targetPos) < 0.6
        fprintf('The drone has reached the target landing position\n');
        break;
    end

    pause(uav.dt);
end