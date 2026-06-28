clear; clc; close all; % Clear workspace


%% Parameter Settings
% entryAngleDeg = rand * 360;  % Entry angle (0=+X, 90=+Y, 180=-X, 270=-Y)
entryAngleDeg = 120;         % Entry angle (0=+X, 90=+Y, 180=-X, 270=-Y)
entryAltitude = 12;         % Entry altitude (m)
entryDistFromEdge = 0.2;    % Distance outside map edge (m)
maxSlopeDeg = 5;            % Max slope for landing (deg)
extraLandingClearance = 1.2;% Extra clearance (m), landing diameter = droneDiameter + extraLandingClearance

% Map Settings
mapSize = 30;               % Map size (m)
gridRes = 0.5;              % Grid resolution (m)

% Drone Settings
droneDiameter = 2;          % Drone diameter (m)

% Simulation Settings
totalTime = 120;            % Total simulation time (s)
simDt = 0.05;               % Simulation time step (s)

% Video recorder settings
RECORD_VIDEO = true;


%% Initialize environment map
map = Autoland_map(mapSize, gridRes);
% view(entryAngleDeg, 45);

%% Initialize drone entry position
entryAngleRad = deg2rad(entryAngleDeg);
entryDir = [cos(entryAngleRad); sin(entryAngleRad)];
center = [mapSize/2; mapSize/2];
oppDir = -entryDir;

t_candidates = [];
if oppDir(1) > 1e-9
    t_candidates = [t_candidates; (mapSize - center(1)) / oppDir(1)];
elseif oppDir(1) < -1e-9
    t_candidates = [t_candidates; -center(1) / oppDir(1)];
end
if oppDir(2) > 1e-9
    t_candidates = [t_candidates; (mapSize - center(2)) / oppDir(2)];
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


%% Initialize mapper
mapper = Autoland_mapper(0.2, droneDiameter, extraLandingClearance);
videoRecorder = Video_recorder('Autoland_Drone.mp4', entryAngleDeg);
steps = totalTime / simDt;

%% Initialize LiDAR sensor
% LiDAR: 96x72 resolution, pitch 0~(-120)deg, roll -45~(-135)deg
lidar = Autoland_lidar(90, 120, 25, 96, 72, 2, map);

disp(['LiDAR initialized: ' num2str(lidar.hRes) 'x' num2str(lidar.vRes) ' resolution, range=' num2str(lidar.beamRange) 'm']);

%% Initialize drone
uav = Autoland_drone(startPos, initialYaw, droneDiameter, simDt);

%% Create visibility control buttons
fig = gcf;
fig.Position = [100 100 1100 720];

btnW = 90; btnH = 22; gap = 25; startY = 640;

% Map elements
uicontrol('Style', 'togglebutton', 'String', 'Map', ...
    'Position', [10 startY btnW btnH], 'Value', 1, ...
    'Callback', @(src,~) map.visTog());

% Mapper elements
uicontrol('Style', 'togglebutton', 'String', 'LiDAR pts', ...
    'Position', [10 startY - gap*2 btnW btnH], 'Value', 1, ...
    'Callback', @(src,~) set(mapper.h_globalMapPlot, 'Visible', getVis(src)));

uicontrol('Style', 'togglebutton', 'String', 'Curr Sites', ...
    'Position', [10 startY-gap*6 btnW btnH], 'Value', 1, ...
    'Callback', @(src,~) set(mapper.h_landingSites, 'Visible', getVis(src)));

uicontrol('Style', 'togglebutton', 'String', 'Hist Sites', ...
    'Position', [10 startY-gap*7 btnW btnH], 'Value', 1, ...
    'Callback', @(src,~) set(mapper.h_allLandingSites, 'Visible', getVis(src)));

uicontrol('Style', 'togglebutton', 'String', 'Patches', ...
    'Position', [10 startY-gap*8 btnW btnH], 'Value', 1, ...
    'Callback', @(src,~) toggleVisible(src, mapper.h_landingPatches));

% Drone elements
droneHandles = [uav.h_body; uav.h_radar; uav.h_arm1; uav.h_arm2; ...
    uav.h_prop1; uav.h_prop2; uav.h_prop3; uav.h_prop4; ...
    uav.h_leg1; uav.h_leg2; uav.h_leg3; uav.h_leg4];

uicontrol('Style', 'togglebutton', 'String', 'Drone', ...
    'Position', [10 startY-gap*10 btnW btnH], 'Value', 1, ...
    'Callback', @(src,~) toggleVisible(src, droneHandles));

uicontrol('Style', 'togglebutton', 'String', 'Path', ...
    'Position', [10 startY-gap*11 btnW btnH], 'Value', 1, ...
    'Callback', @(src,~) set(uav.h_path, 'Visible', getVis(src)));

uicontrol('Style', 'togglebutton', 'String', 'Scan pts', ...
    'Position', [10 startY-gap*12 btnW btnH], 'Value', 1, ...
    'Callback', @(src,~) set(uav.h_cloud_dots, 'Visible', getVis(src)));

uicontrol('Style', 'togglebutton', 'String', 'Target', ...
    'Position', [10 startY-gap*13 btnW btnH], 'Value', 1, ...
    'Callback', @(src,~) set(uav.h_target_marker, 'Visible', getVis(src)));


landingSites = [];

for t = 1:steps
    currentScanPoints = lidar.scan(uav.Position, uav.Yaw);
    mapper.updateMap(currentScanPoints);

    % Analyze terrain more frequently
    if mod(t, 5) == 0
        landingSites = mapper.analyzeTerrain(maxSlopeDeg);
    end

    uav.update(currentScanPoints, lidar.F_terrain, mapper.AllLandingSites);

    if mod(t, 2) == 0
        mapper.renderMap();
        uav.render(currentScanPoints);
        % lidar.updateBeams(uav.Position, uav.Yaw);
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

disp(' ');

%% Local functions
function vis = getVis(src)
if src.Value == 1, vis = 'on'; else, vis = 'off'; end
end

function toggleVisible(src, hArray)
if ~isvalid(src), return; end
vis = 'on'; if src.Value == 0, vis = 'off'; end
if isempty(hArray), return; end
for i = 1:length(hArray)
    if isvalid(hArray(i))
        set(hArray(i), 'Visible', vis);
    end
end
end
