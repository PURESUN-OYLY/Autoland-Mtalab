clear; clc; close all; % Clear workspace

RECORD_VIDEO = false;

% Initialize Decoupled Forest Terrain
mapEnvironment = Autoland_map();

% Buid map, trees ratio input, higher will be create mor trees
mapEnvironment.generateEnvironment();

mapper = Autoland_mapper(0.5);

% Initialize Video Recorder
videoRecorder = Video_recorder('Autoland_Drone.mp4');

% Initialize Upgraded Autoland Drone Model
uav = Autoland_drone([5; 5; 10]);
% uav = Autoland_drone(mapEnvironment.startPos);
totalTime = 120;
steps = totalTime / uav.dt;

% Initialize Decoupled Telemetry LiDAR Array
lidarSensor = Autoland_lidar(120, 60, 25, 192, 96, 2, true);

% Initialize LiDAR System with Real Terrain Data
lidarSensor.setTerrain(mapEnvironment.X, mapEnvironment.Y, mapEnvironment.Z_ground);
disp(['LiDAR system initialized: Scan range = ' num2str(lidarSensor.beamRange) ' m']);

% Initialize Visualization Environment
axis equal; % Maintain aspect ratio
axis vis3d; % Enable 3D visualization

targetPos = [20; 20; 10];

% Real-time Closed-Loop Loop
for t = 1:steps
    currentScanPoints = lidarSensor.getScanCloud(uav.Position, uav.Yaw, mapEnvironment.treeLocations);

    mapper.updateMap(currentScanPoints);

    % Update the drone's position
    uav.update(targetPos, mapper.GlobalMap, lidarSensor.F_terrain);
    % uav.update(mapEnvironment.targetPos, currentScanPoints, lidarSensor.F_terrain);

    % render the scene every 2 steps, olnly for faster visualization
    if mod(t, 2) == 0
        % Update the map
        mapper.renderMap();

        % Render the scene
        uav.render(currentScanPoints);
        lidarSensor.updateBeams(uav.Position, uav.Yaw);
        drawnow limitrate;

        if RECORD_VIDEO
            videoRecorder.captureFrame();
        end
    end

    %  check if the drone has reached the target
    if norm(uav.Position - targetPos) < 0.6
        disp('Drone has safely reached the target!');
        break;
    end

    % display progress in percentage
    fprintf('Progress: %.2f%%\r', t / steps * 100);
    
    % pause for a short time to allow visualization
    pause(uav.dt / 10);
end

%% 5. Close Video Recorder
if RECORD_VIDEO
    videoRecorder.closeVideo();
end
