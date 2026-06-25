classdef Autoland_mapper < handle
    properties
        GlobalMap = [];
        GridSize = 0.4;
        h_globalMapPlot
        LandingSites = [];
        AllLandingSites = [];
        h_landingSites = []
        h_allLandingSites = []
        h_landingPatches = []
        droneDiameter = 2.4;
        extraClearance = 0.6;
    end
    
    methods
        function obj = Autoland_mapper(gridSize, droneDiameter, extraClearance)
            if nargin >= 1 && ~isempty(gridSize), obj.GridSize = gridSize; end
            if nargin >= 2 && ~isempty(droneDiameter), obj.droneDiameter = droneDiameter; end
            if nargin >= 3 && ~isempty(extraClearance), obj.extraClearance = extraClearance; end
            hold on;
            obj.h_globalMapPlot = plot3(NaN, NaN, NaN, '.', 'MarkerSize', 3);
            obj.h_landingSites = scatter3(NaN, NaN, NaN, 60, 'g', 'p', 'filled', ...
                'MarkerEdgeColor', [0 0.4 0], 'LineWidth', 1);
            obj.h_allLandingSites = scatter3(NaN, NaN, NaN, 80, 'b', 'h', 'filled', ...
                'MarkerEdgeColor', [0 0.2 0.6], 'LineWidth', 1.5);
        end
        
        function updateMap(obj, newPoints)
            if isempty(newPoints), return; end
            combinedPoints = [obj.GlobalMap; newPoints];
            uGrid = round(combinedPoints / obj.GridSize);
            [~, uniqueIdx, ~] = unique(uGrid, 'rows', 'stable');
            obj.GlobalMap = combinedPoints(uniqueIdx, :);
        end
        
        function landingSites = analyzeTerrain(obj, maxSlopeDeg)
            landingSites = [];
            obj.LandingSites = [];
            if isempty(obj.GlobalMap) || size(obj.GlobalMap, 1) < 30
                return;
            end
            
            minRadius = (obj.droneDiameter + obj.extraClearance) / 2;
            
            % Extract ground points using lowest-Z method
            allZ = obj.GlobalMap(:, 3);
            z_min = min(allZ);
            z_thresh = z_min + 1.5;  % within 1.5m of lowest point
            groundMask = allZ <= z_thresh;
            groundPoints = obj.GlobalMap(groundMask, :);
            
            if size(groundPoints, 1) < 15
                return;
            end
            
            % 2D grid analysis
            gridRes = 0.3;
            xMin = floor(min(groundPoints(:,1)) / gridRes) * gridRes;
            xMax = ceil(max(groundPoints(:,1)) / gridRes) * gridRes;
            yMin = floor(min(groundPoints(:,2)) / gridRes) * gridRes;
            yMax = ceil(max(groundPoints(:,2)) / gridRes) * gridRes;
            xGrid = xMin:gridRes:xMax;
            yGrid = yMin:gridRes:yMax;
            nx = length(xGrid); ny = length(yGrid);
            
            slopeMap = inf(ny, nx);
            zMap = nan(ny, nx);
            validCount = 0;
            
            for ix = 1:nx
                for iy = 1:ny
                    cx = xGrid(ix); cy = yGrid(iy);
                    dist = sqrt((groundPoints(:,1) - cx).^2 + (groundPoints(:,2) - cy).^2);
                    nearby = dist < gridRes * 2.0;
                    if sum(nearby) >= 4
                        pts = groundPoints(nearby, :);
                        A = [pts(:,1) - mean(pts(:,1)), pts(:,2) - mean(pts(:,2)), ones(size(pts,1), 1)];
                        b = pts(:,3) - mean(pts(:,3));
                        coeffs = A \ b;
                        nx_plane = coeffs(1); ny_plane = coeffs(2); nz_plane = -1;
                        n_norm = sqrt(nx_plane^2 + ny_plane^2 + nz_plane^2);
                        slopeRad = acos(abs(nz_plane) / n_norm);
                        slopeDeg = rad2deg(slopeRad);
                        slopeMap(iy, ix) = slopeDeg;
                        zMap(iy, ix) = mean(pts(:,3));
                        validCount = validCount + 1;
                    end
                end
            end
            
            if validCount < 8
                return;
            end
            
            validMask = slopeMap < maxSlopeDeg;
            if ~any(validMask(:))
                return;
            end
            
            % Connected component labeling (BFS)
            labeled = zeros(ny, nx);
            label = 0;
            for iy = 1:ny
                for ix = 1:nx
                    if validMask(iy, ix) && labeled(iy, ix) == 0
                        label = label + 1;
                        queue = zeros(ny * nx, 2);
                        queue(1, :) = [iy, ix];
                        qTail = 1; qHead = 1;
                        labeled(iy, ix) = label;
                        while qHead <= qTail
                            cy = queue(qHead, 1); cx = queue(qHead, 2);
                            qHead = qHead + 1;
                            for dy = -1:1
                                for dx = -1:1
                                    if dy == 0 && dx == 0, continue; end
                                    ny2 = cy + dy; nx2 = cx + dx;
                                    if ny2 >= 1 && ny2 <= ny && nx2 >= 1 && nx2 <= nx
                                        if validMask(ny2, nx2) && labeled(ny2, nx2) == 0
                                            qTail = qTail + 1;
                                            queue(qTail, :) = [ny2, nx2];
                                            labeled(ny2, nx2) = label;
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
            
            sites = [];
            for i = 1:label
                pixIdx = find(labeled == i);
                if length(pixIdx) < 4
                    continue;
                end
                area = length(pixIdx) * gridRes^2;
                if area < pi * minRadius^2
                    continue;
                end
                
                [rowIdx, colIdx] = ind2sub([ny, nx], pixIdx);
                cx = mean(xGrid(colIdx));
                cy = mean(yGrid(rowIdx));
                cz = mean(zMap(pixIdx));
                if isnan(cz), continue; end
                maxSlope = max(slopeMap(pixIdx));
                radius = sqrt(area / pi);
                
                sites = [sites; cx, cy, cz, radius, maxSlope, area];
            end
            
            landingSites = sites;
            obj.LandingSites = sites;
            
            % Merge new sites with historical sites, avoiding duplicates
            minSep = (obj.droneDiameter + obj.extraClearance) * 0.5;
            if ~isempty(obj.AllLandingSites) && ~isempty(sites)
                newSites = [];
                for i = 1:size(sites, 1)
                    isDuplicate = false;
                    for j = 1:size(obj.AllLandingSites, 1)
                        if norm(sites(i, 1:2) - obj.AllLandingSites(j, 1:2)) < minSep
                            isDuplicate = true;
                            break;
                        end
                    end
                    if ~isDuplicate
                        newSites = [newSites; sites(i, :)];
                    end
                end
                sites = newSites;
            end
            
            if ~isempty(sites)
                obj.AllLandingSites = [obj.AllLandingSites; sites];
            end
            
            landingSites = sites;
            obj.LandingSites = sites;
        end
        
        function renderMap(obj)
            if ~isempty(obj.GlobalMap)
                set(obj.h_globalMapPlot, 'XData', obj.GlobalMap(:, 1), ...
                    'YData', obj.GlobalMap(:, 2), 'ZData', obj.GlobalMap(:, 3));
            end
            
            if ~isempty(obj.LandingSites)
                set(obj.h_landingSites, 'XData', obj.LandingSites(:, 1), ...
                    'YData', obj.LandingSites(:, 2), 'ZData', obj.LandingSites(:, 3) + 0.3);
            else
                set(obj.h_landingSites, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
            end
            
            if ~isempty(obj.AllLandingSites)
                set(obj.h_allLandingSites, 'XData', obj.AllLandingSites(:, 1), ...
                    'YData', obj.AllLandingSites(:, 2), 'ZData', obj.AllLandingSites(:, 3) + 0.3);
            else
                set(obj.h_allLandingSites, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
            end
            
            % Clear old patches
            for i = 1:length(obj.h_landingPatches)
                if isvalid(obj.h_landingPatches(i))
                    delete(obj.h_landingPatches(i));
                end
            end
            obj.h_landingPatches = [];
            
            if ~isempty(obj.AllLandingSites)
                theta = linspace(0, 2*pi, 24);
                for i = 1:size(obj.AllLandingSites, 1)
                    r = obj.AllLandingSites(i, 4);
                    cx = obj.AllLandingSites(i, 1);
                    cy = obj.AllLandingSites(i, 2);
                    cz = obj.AllLandingSites(i, 3) + 0.15;
                    x = cx + r * cos(theta);
                    y = cy + r * sin(theta);
                    z = cz * ones(size(theta));
                    h = fill3(x, y, z, [1, 0.95, 0.25], 'FaceAlpha', 0.35, ...
                        'EdgeColor', [0.85, 0.8, 0.1], 'LineWidth', 1.5);
                    obj.h_landingPatches = [obj.h_landingPatches; h];
                end
            end
        end
    end
end
