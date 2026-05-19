clear; clc; close all; % Clear workspace

%% 0. Global Parameters(for configuration)
totalTime = 120;
steps = totalTime / uav.dt;
axis equal; 

%% 1. Initialize Decoupled Forest Terrain
mapEnvironment = Autoland_map();
mapEnvironment.generateEnvironment(0.015); 

%% 2. Initialize Upgraded Autoland Drone Model
uav = Autoland_drone([5; 5; 10]);

%% 3. Initialize Decoupled Telemetry LiDAR Array
lidarSensor = Autoland_lidar(120, 60, 25, 192, 96, 4, false); 

%% 4. Initialize LiDAR System with Real Terrain Data
surfObj = findobj(gca, 'Type', 'Surface');
if ~isempty(surfObj)
    % Give real terrain data to LiDAR system
    lidarSensor.setTerrain(surfObj(1).XData, surfObj(1).YData, surfObj(1).ZData);
    disp(['LiDAR system initialized: Scan range = ' num2str(lidarSensor.beamRange) ' m']);
end

%% 5. Real-time Closed-Loop Loop
for t = 1:steps
    currentScanPoints = lidarSensor.getScanCloud(uav.Position, uav.Yaw, mapEnvironment.treeLocations);
    uav.update(mapEnvironment.targetPos, currentScanPoints);
    
    % render the scene every 2 steps, olnly for faster visualization
    if mod(t, 2) == 0
        uav.render(currentScanPoints);
        lidarSensor.updateBeams(uav.Position, uav.Yaw);
        drawnow limitrate;
    end
    
    %  check if the drone has reached the target
    if norm(uav.Position - mapEnvironment.targetPos) < 0.6
        disp(['Drone has safely reached the target!']);
        break;
    end

    % display progress bar
    fprintf('Progress: %.2f%%\r', t / steps * 100);
    pause(uav.dt);
end