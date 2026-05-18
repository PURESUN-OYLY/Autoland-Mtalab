clear; clc; close all; % Clear the workspace

%% 0. Initialize Camera for recording
videoName = 'Autoland_forest.mp4';
video = VideoWriter(videoName, 'MPEG-4');
video.FrameRate = 30;
video.Quality = 100;
open(video);

%% 1. Initialize Decoupled Forest Terrain Context Structure File
mapEnvironment = Autoland_map();
mapEnvironment.generateEnvironment(0.015); % Plant trees density setting input parameter

%% 2. Initialize Upgraded Autoland Drone Model Object
% Drone begins position trace initialized securely inside boundaries
uav = Autoland_drone([5; 5; 10]);

%% 3. Initialize Decoupled Telemetry LiDAR Array Module Object
lidarSensor = Autoland_lidar(120, 60);

%% 4. Define Integration Simulation Parameters
totalTime = 20;
steps = totalTime / uav.dt;
axis equal; % Maintain equal aspect ratio for 3D visualization

%% 5. Real-time Closed-Loop Simulation Loop
for t = 1:steps
    % --- Controller and perceives calculations step updating ---
    % Passes live position inputs and map obstacle coordinates directly into the tracking handler
    uav.update(mapEnvironment.targetPos, mapEnvironment.treeLocations);
    
    % --- Visual Layer Refresher ---
    if mod(t, 2) == 0
        % Calls visualization drawing methods isolated cleanly within model definitions
        uav.render();
        lidarSensor.updateBeams(uav.Position, uav.Yaw);
        drawnow limitrate;
    end
    
    % Destination check metric boundary processing
    if norm(uav.Position - mapEnvironment.targetPos) < 0.6
        fprintf('The drone has safely navigated around obstacles and reached the target area.\n');
        break;
    end
    
    pause(uav.dt);
    frame = getframe(gcf);
    writeVideo(video, frame);
    disp(['Frame ', num2str(t), ' recorded, total ', num2str(steps), ' frames.']);
end

%% 6. Close Camera for recording
close(video);
disp('Video recording completed.');
