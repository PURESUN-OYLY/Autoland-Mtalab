clear; clc; close all; % Clear workspace

RECORD_VIDEO = false;

% ========== Parameter Settings ==========
entryAngleDeg = rand * 360;  % Entry angle (0=+X, 90=+Y, 180=-X, 270=-Y)
entryAltitude = 12;          % Entry altitude (m)
entryDistFromEdge = 0.2;     % Distance outside map edge (m)
maxSlopeDeg = 5;             % Max slope for landing (deg)
extraLandingClearance = 1.2; % Extra clearance (m), landing diameter = droneDiameter + extraLandingClearance
% =========================================

mapEnvironment = Autoland_map();
mapEnvironment.generateEnvironment();
fieldSize = mapEnvironment.fieldSize;

entryAngleRad = deg2rad(entryAngleDeg);
entryDir = [cos(entryAngleRad); sin(entryAngleRad)];
center = [fieldSize/2; fieldSize/2];
oppDir = -entryDir;

t_candidates = [];
if oppDir(1) > 1e-9
    t_candidates = [t_candidates; (fieldSize - center(1)) / oppDir(1)];
elseif oppDir(1) < -1e-9
    t_candidates = [t_candidates; -center(1) / oppDir(1)];
end
if oppDir(2) > 1e-9
    t_candidates = [t_candidates; (fieldSize - center(2)) / oppDir(2)];
elseif oppDir(2) < -1e-9
    t_candidates = [t_candidates; -center(2) / oppDir(2)];
end

if ~isempty(t_candidates)
    t_boundary = min(t_candidates(t_candidates > 0));
    boundaryPos = center + oppDir * t_boundary;
    startPosXY = boundaryPos + oppDir * entryDistFromEdge;
else
    startPosXY = [0; fieldSize/2];
end

startPos = [startPosXY; entryAltitude];
initialYaw = entryAngleRad;

uav = Autoland_drone(startPos, initialYaw);
droneDiameter = uav.armLength * 2 + uav.propRadius * 2;
disp(['Drone diameter: ' num2str(droneDiameter) 'm']);
mapper = Autoland_mapper(0.2, droneDiameter, extraLandingClearance);
videoRecorder = Video_recorder('Autoland_Drone.mp4');
totalTime = 120;
steps = totalTime / uav.dt;

% LiDAR: 192x144 resolution, pitch 0~(-120)deg, roll -45~(-135)deg
lidarSensor = Autoland_lidar(90, 120, 25, 129, 144, 2);
lidarSensor.setTerrain(mapEnvironment.X, mapEnvironment.Y, mapEnvironment.Z_ground);
lidarSensor.setObstacles(mapEnvironment.rockLocations, mapEnvironment.stumpPos, mapEnvironment.bushLocations);
disp(['LiDAR initialized: ' num2str(lidarSensor.hRes) 'x' num2str(lidarSensor.vRes) ' resolution, range=' num2str(lidarSensor.beamRange) 'm']);

axis equal; axis vis3d;

landingSites = [];

for t = 1:steps
    currentScanPoints = lidarSensor.getScanCloud(uav.Position, uav.Yaw, mapEnvironment.treeLocations);
    mapper.updateMap(currentScanPoints);

    % Analyze terrain more frequently
    if mod(t, 5) == 0
        landingSites = mapper.analyzeTerrain(maxSlopeDeg);
    end

    uav.update(currentScanPoints, lidarSensor.F_terrain, mapper.AllLandingSites);

    if mod(t, 2) == 0
        mapper.renderMap();
        uav.render(currentScanPoints);
        % lidarSensor.updateBeams(uav.Position, uav.Yaw);
        drawnow limitrate;
        if RECORD_VIDEO
            videoRecorder.captureFrame();
        end
    end

    if strcmp(uav.State, 'LANDED')
        disp('Simulation complete: Drone has landed successfully.');
        break;
    end

    fprintf('Progress: %.2f%% | State: %s | Pos: [%.1f, %.1f, %.1f] | LandingSites: %d\r', ...
        t / steps * 100, uav.State, uav.Position(1), uav.Position(2), uav.Position(3), size(landingSites,1));
    pause(uav.dt / 10);
end

if RECORD_VIDEO
    videoRecorder.closeVideo();
end
