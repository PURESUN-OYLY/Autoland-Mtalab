classdef Autoland_map < handle
    properties
        fieldSize = 100;    % Map size (m)
        gridRes = 0.5;      % Grid resolution (m)
        X, Y, Z_ground;     % Map surface grids
        validSlopeMask;     % Logical matrix for terrain properties
        treeLocations = []; % Matrix storing [tx, ty, trunkR, trunkH, h_base]
        targetPos;          % Calculated landing destination vector
        startPos;           % Starting position vector
        Fterrain;           % Terrain height function
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
            obj.Fterrain = scatteredInterpolant(obj.X(:), obj.Y(:), obj.Z_ground(:), 'linear', 'nearest');
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
                    % obj.treeLocations = [obj.treeLocations; tx, ty, trunkR, trunkH, h_base];
                    canopyR = 2.5 + rand * 1.5;

                    obj.treeLocations=[obj.treeLocations, tx, ty, trunkR, trunkH, h_base, canopyR];

                    % Construct cylindrical model representations for tree trunk
                    [cX, cY, cZ] = cylinder([1, 1], 12);
                    surf(cX*trunkR + tx, cY*trunkR + ty, cZ*trunkH + h_base, 'FaceColor', [0.45, 0.25, 0.05], 'EdgeColor', 'none');

                    % Construct spherical model representations for tree canopy
                    [sx, sy, sz] = sphere(16);
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

                scatter3(                     obj.X(connectedRegions), obj.Y(connectedRegions), obj.Z_ground(connectedRegions) + 0.1, 20,                     'y', 'filled', 'MarkerEdgeAlpha', 0.3, 'MarkerFaceAlpha', 0.3);

                CC = bwconncomp(connectedRegions);
                numRegions = CC.NumObjects;
                centers = zeros(numRegions,2);
                for k = 1:numRegions
                    pixels = CC.PixelIdxList{k};
                    [rr, cc] = ind2sub(size(connectedRegions), pixels);
                    xx = obj.X(sub2ind(size(obj.X), rr, cc));
                    yy = obj.Y(sub2ind(size(obj.Y), rr, cc));
                    centers(k,:)=[mean(xx), mean(yy)];
                end

                maxDist=0;
                best1=1;
                best2=2;

                for i = 1:numRegions
                    for j = i+1:numRegions
                        d = norm(centers(i,:) - centers(j,:));
                        if d > maxDist
                            maxDist = d;
                            best1=i;
                            best2=j;
                        end
                    end
                end

                startCenter=centers(best1,:);
                targetCenter=centers(best2,:);
                z1=obj.Fterrain(startCenter(1), startCenter(2));
                z2=obj.Fterrain(targetCenter(1), targetCenter(2));
                obj.startPos=[startCenter(1); startCenter(2); z1+8];
                obj.targetPos=[targetCenter(1); targetCenter(2); z2+3];
            else
                obj.startPos = [5; 5; 10];
                obj.targetPos = [80; 80; 13];
            end

            % Draw simulation markers
            % plot3(5, 5, interp2(obj.X, obj.Y, obj.Z_ground, 5, 5)+1, 'bp', 'MarkerSize', 15, 'MarkerFaceColor', 'b');
            plot3(obj.startPos(1), obj.startPos(2), obj.startPos(3), 'bp', 'MarkerSize',15, 'MarkerFaceColor','b');
            text(obj.startPos(1), obj.startPos(2), obj.startPos(3)+3, 'Start point', 'FontWeight','bold');
            plot3(obj.targetPos(1), obj.targetPos(2), obj.targetPos(3), 'rp', 'MarkerSize',15, 'MarkerFaceColor','r');
            text(obj.targetPos(1), obj.targetPos(2), obj.targetPos(3)+3, 'Landing point', 'FontWeight','bold');
            % text(5, 5, interp2(obj.X, obj.Y, obj.Z_ground, 5, 5)+4, 'Start point', 'FontWeight', 'bold');
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Height (m)');
            view(-45, 35); axis tight;
        end
    end
end