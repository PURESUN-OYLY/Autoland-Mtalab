classdef Autoland_map < handle
    properties
        fieldSize = 100;    % Map size (m)
        gridRes = 0.5;      % Grid resolution (m)
        X, Y, Z_ground      % Map surface grids
        validSlopeMask      % Logical matrix for terrain properties
        treeLocations = []; % Matrix storing [tx, ty, trunkR, trunkH, h_base]
        targetPos           % Calculated landing destination vector
    end
    
    methods
        function obj = Autoland_map()
            [obj.X, obj.Y] = meshgrid(0:obj.gridRes:obj.fieldSize);
            
            % Generate terrain elevation values
            obj.Z_ground = 4 * sin(obj.X/15) .* cos(obj.Y/15) + 2 * cos(obj.X/25) + 0.3 * randn(size(obj.X))*0.05;
            obj.Z_ground = obj.Z_ground - min(obj.Z_ground, [], 'all');
            
            % Slope processing gradient calculations
            [dzdx, dzdy] = gradient(obj.Z_ground, obj.gridRes);
            slopeRad = atan(sqrt(dzdx.^2 + dzdy.^2));
            slopeDeg = rad2deg(slopeRad);
            obj.validSlopeMask = slopeDeg < 5;
        end
        
        function generateEnvironment(obj, treeDensity)
            % Render environment background canvas
            surf(obj.X, obj.Y, obj.Z_ground, 'EdgeColor', 'none', 'FaceAlpha', 0.8);
            colormap(summer); hold on; light; lighting gouraud;
            
            % Plant trees iteratively across the terrain map
            numTrees = round(obj.fieldSize^2 * treeDensity);
            rng(20);
            
            for i = 1:numTrees
                tx = rand * obj.fieldSize; 
                ty = rand * obj.fieldSize;
                
                if norm([tx, ty] - [5, 5]) > 5
                    h_base = interp2(obj.X, obj.Y, obj.Z_ground, tx, ty);
                    trunkR = 0.6 + rand * 0.3;
                    trunkH = 5 + rand * 3;
                    
                    % Track parameter indices for collision checks
                    obj.treeLocations = [obj.treeLocations; tx, ty, trunkR, trunkH, h_base];
                    
                    % Construct cylindrical model representations for tree trunk
                    [cX, cY, cZ] = cylinder([1, 1], 12);
                    surf(cX*trunkR + tx, cY*trunkR + ty, cZ*trunkH + h_base, 'FaceColor', [0.45, 0.25, 0.05], 'EdgeColor', 'none');
                    
                    % Construct spherical model representations for tree canopy
                    [sx, sy, sz] = sphere(16); 
                    canopyR = 2.5 + rand * 1.5;
                    surf(sx*canopyR + tx, sy*canopyR + ty, sz*canopyR + h_base + trunkH, 'FaceColor', [0.1, 0.4, 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
                    
                    % Exclude localized ground perimeter matching the radius profile
                    treeDist = sqrt((obj.X - tx).^2 + (obj.Y - ty).^2);
                    obj.validSlopeMask(treeDist < (trunkR + 1.5)) = 0;
                end
            end
            
            % Isolate valid contiguous elements 
            minPixels = ceil(1.0 / obj.gridRes^2);
            connectedRegions = bwpropfilt(obj.validSlopeMask, 'Area', [minPixels, inf]);
            [r, c] = find(connectedRegions);
            
            if ~isempty(r)
                scatter3(obj.X(connectedRegions), obj.Y(connectedRegions), obj.Z_ground(connectedRegions)+0.1, ...
                    20, 'y', 'filled', 'MarkerEdgeAlpha', 0.3, 'MarkerFaceAlpha', 0.3);
                
                targetIdx = round(length(r)/2);
                targetX = obj.X(r(targetIdx), c(targetIdx));
                targetY = obj.Y(r(targetIdx), c(targetIdx));
                targetH = obj.Z_ground(r(targetIdx), c(targetIdx));
                obj.targetPos = [targetX; targetY; targetH + 3];
            else
                obj.targetPos = [80; 80; 13];
            end
            
            % Draw simulation markers
            plot3(5, 5, interp2(obj.X, obj.Y, obj.Z_ground, 5, 5)+1, 'bp', 'MarkerSize', 15, 'MarkerFaceColor', 'b');
            text(5, 5, interp2(obj.X, obj.Y, obj.Z_ground, 5, 5)+4, 'Start point', 'FontWeight', 'bold');
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Height (m)');
            view(-45, 35); axis tight;
        end
    end
end