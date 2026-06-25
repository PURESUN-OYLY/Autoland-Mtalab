classdef Autoland_map < handle
    properties
        % Map parameters
        fieldSize = 30;        % Map size in meters
        gridRes = 0.5;         % Grid resolution in meters
        X, Y, Z_ground;        % Terrain grid
        validSlopeMask;        % Valid slope area mask
        emptyLandMask;         % Empty land mask (no obstacles)
        Fterrain;              % Terrain height interpolation function

        % Obstacle coordinates storage matrix
        treeLocations = [];    % [tx, ty, trunkR, trunkH, h_base, canopyR, branchNum]
        stumpPos = [];         % Tree stump [sx, sy, r_stump, h_stump, h_base]
        rockLocations = [];    % Rock [rx, ry, r_min, r_max, h_rock, h_base]
        bushLocations = [];    % Bush grass [bx, by, r_bush, h_bush, h_base]

        % Render boolean switches
        renderTerrain    = true;
        renderTree       = true;
        renderBranch     = true;
        renderRock       = true;
        renderBushGrass  = true;
        renderMarkers    = true;


        %% Render handles
        h_terrain
        h_trees = []
        h_rocks = []
        h_bushes = []
        h_markers = []
    end

    methods
        function obj = Autoland_map()
            disp('Building basic terrain grid...');
            % Generate basic terrain grid
            [obj.X, obj.Y] = meshgrid(0:obj.gridRes:obj.fieldSize);

            % Generate basic terrain, with flat hills and small-scale noise
            % Base terrain with flat hills (preserving overall base height)
            Z_base = 1.0 * sin(obj.X/8) .* cos(obj.Y/8) + 0.7 * cos(obj.X/12) + 0.2 * sin(obj.Y/10);

            % Add 3 local steep hills to the terrain
            rng(10); % Fix random seed for consistent terrain
            numHills = 3;
            Z_hills = zeros(size(obj.X));
            for i = 1:numHills
                hx = 5 + rand()*20;    % Hill center X
                hy = 5 + rand()*20;    % Hill center Y
                hillHeight = 2.8 + rand()*1.2; % Hill height 2.8~4m
                hillRadius = 3.0 + rand()*1.5; % Hill radius 3~4.5m (smaller radius means steeper slope)
                dist = sqrt((obj.X - hx).^2 + (obj.Y - hy).^2);
                % Gaussian hill shape, with natural transition at edges and peak at center
                Z_hills = Z_hills + hillHeight * exp(-(dist.^2) / (2*hillRadius^2));
            end

            % Add small-scale noise to the terrain
            Z_noise = 0.08 * randn(size(obj.X));

            % Combine all components to form the final terrain grid
            obj.Z_ground = Z_base + Z_hills + Z_noise;
            obj.Z_ground = obj.Z_ground - min(obj.Z_ground, [], 'all');
            % Normalize terrain height to make the lowest point at 0
            obj.Z_ground = obj.Z_ground - min(obj.Z_ground, [], 'all');

            % Calculate slope mask (valid slope area < 5°)
            [dzdx, dzdy] = gradient(obj.Z_ground, obj.gridRes);
            slopeRad = atan(sqrt(dzdx.^2 + dzdy.^2));
            slopeDeg = rad2deg(slopeRad);
            obj.validSlopeMask = slopeDeg < 5;

            % Initialize empty land mask (no obstacles)
            obj.emptyLandMask = true(size(obj.validSlopeMask));

            % Create terrain height interpolation function
            obj.Fterrain = scatteredInterpolant(obj.X(:), obj.Y(:), obj.Z_ground(:), 'linear', 'nearest');
        end

        % Main environment generator function, step-by-step execution
        function generateEnvironment(obj)
            rng(42); % Fix random seed for consistent environment generation
            hold on; grid off;

            % Render basic terrain grass
            disp('Render basic terrain grass');
            if obj.renderTerrain
                obj.h_terrain = surf(obj.X, obj.Y, obj.Z_ground, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
                colormap(summer); light; lighting gouraud;
            end

            % Render trees
            disp('Render trees');
            if obj.renderTree
                numTrees = randi([8,12]); % Random 8-12 trees
                for i = 1:numTrees
                    % Make sure trees are not too close to the edges
                    tx = 5 + rand() * 20;
                    ty = 5 + rand() * 20;
                    h_base = obj.Fterrain(tx, ty); % Tree base height

                    % Tree trunk parameters
                    trunkR = 0.4 + rand() * 0.4;
                    trunkH = 4 + rand() * 4;
                    branchNum = randi([4,7]); % Branch number per tree
                    canopyR = 1.8 + rand() * 1.2;

                    % Store tree data
                    obj.treeLocations = [obj.treeLocations; tx, ty, trunkR, trunkH, h_base, canopyR, branchNum];

                    % Draw tree trunk cylinder
                    [cX,cY,cZ] = cylinder(trunkR,16);
                    surf(cX+tx, cY+ty, cZ*trunkH + h_base, ...
                        'FaceColor', [0.42,0.24,0.06], 'EdgeColor','none');

                    % Draw tree branches
                    for b = 1:branchNum
                        branchLen = 1.2 + rand()*1.0;
                        branchR = trunkR * (0.2 + rand()*0.3);
                        branchAngleX = rand()*2*pi;
                        branchAngleZ = pi/4 + rand()*pi/3;
                        branchBaseZ = h_base + trunkH * (0.4 + rand()*0.5);

                        % Transform branch coordinates to tree trunk
                        [brX,brY,brZ] = cylinder(branchR,8);
                        brZ = brZ * branchLen;
                        rotMatX = [cos(branchAngleX), -sin(branchAngleX),0;
                            sin(branchAngleX), cos(branchAngleX),0;
                            0,0,1];
                        rotMatZ = [1,0,0;
                            0,cos(branchAngleZ),-sin(branchAngleZ);
                            0,sin(branchAngleZ),cos(branchAngleZ)];
                        rotAll = rotMatX * rotMatZ;
                        pts = rotAll * [brX(:)'; brY(:)'; brZ(:)'];
                        brX_rot = reshape(pts(1,:), size(brX));
                        brY_rot = reshape(pts(2,:), size(brY));
                        brZ_rot = reshape(pts(3,:), size(brZ));

                        surf(brX_rot + tx, brY_rot + ty, brZ_rot + branchBaseZ, ...
                            'FaceColor', [0.38,0.21,0.04], 'EdgeColor','none');

                        % Draw leaf cluster at branch end
                        leafR = canopyR * (0.4 + rand()*0.6);
                        [sx,sy,sz] = sphere(12);
                        surf(sx*leafR + tx + brX_rot(end,end), ...
                            sy*leafR + ty + brY_rot(end,end), ...
                            sz*leafR + branchBaseZ + brZ_rot(end,end), ...
                            'FaceColor', [0.08,0.45,0.12], 'EdgeColor','none','FaceAlpha',0.6);
                    end

                    % Draw tree canopy top
                    [sx,sy,sz] = sphere(20);
                    surf(sx*canopyR + tx, sy*canopyR + ty, sz*canopyR + h_base + trunkH, ...
                        'FaceColor', [0.12,0.52,0.15], 'EdgeColor','none','FaceAlpha',0.5);

                    % Tree area mask
                    treeDist = sqrt((obj.X - tx).^2 + (obj.Y - ty).^2);
                    obj.validSlopeMask(treeDist < (trunkR + 2.0)) = 0;
                    obj.emptyLandMask(treeDist < (trunkR + 2.0)) = 0;
                end
            end

            disp('Render bush grass');
            obj.build_bushgrass(randi([12,18]));

            disp('Render rocks');
            obj.build_rock(randi([8, 15]));

            disp('Render fallen dead trunks with branches');
            obj.build_fallen_trunk(10, 15, 20);

            % Check terrain area constraints
            % disp('==== Check terrain area constraints');
            % totalPixel = numel(obj.emptyLandMask);
            % emptyPixel = sum(obj.emptyLandMask(:));
            % slopeValidPixel = sum(obj.validSlopeMask(:));
            % emptyRatio = emptyPixel / totalPixel;
            % slopeValidRatio = slopeValidPixel / totalPixel;
            % fprintf('Empty land ratio：%.2f %% (required≥50%%)\n', emptyRatio*100);
            % fprintf('Slope valid<5° ratio：%.2f %% (required≥10%%)\n', slopeValidRatio*100);
            
            % Draw valid slope area yellow markers
            % scatter3(obj.X(obj.validSlopeMask), obj.Y(obj.validSlopeMask), ...
            %     obj.Z_ground(obj.validSlopeMask)+0.05, 80, 'y', 'filled', ...
            %     'MarkerFaceAlpha', 0.3, 'MarkerEdgeAlpha', 0.3);


            %% Step9: Draw coordinate axis and view angle
            disp('Draw coordinate axis and view angle');
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Altitude (m)');
            xlim([0 obj.fieldSize]); ylim([0 obj.fieldSize]);
            view(-40, 32); axis equal tight;
            hold off;
            disp('Map environment generated successfully.');
        end
    end
    methods (Access = private)
        % Private methods

        function build_bushgrass(obj, bush_number)
            if ~obj.renderBushGrass
                return;
            end
            baseRes = 10;
            for iBush = 1:bush_number
                % The location of the bush on the ground plane
                bx = rand() * 30;
                by = rand() * 30;
                groundZ = obj.Fterrain(bx, by);

                % The size of the bush
                bushRadiusX = 0.7 + rand()*0.9;
                bushRadiusZ = 0.6 + rand()*1.0;
                leafTotal = randi([10,20]); % The number of leaves in the bush

                % Generate each leaf in the bush
                for leafIdx = 1:leafTotal
                    % Random offset of the leaf in the ellipsoid of the bush
                    thetaRand = rand() * 2*pi;
                    phiRand  = rand() * pi;
                    distRand = rand()^0.5; % The distance of the leaf from the center of the ellipsoid

                    lOffX = distRand * bushRadiusX * sin(phiRand) * cos(thetaRand);
                    lOffY = distRand * bushRadiusX * sin(phiRand) * sin(thetaRand);
                    lOffZ = distRand * bushRadiusZ * cos(phiRand);

                    % The size of the leaf
                    leafW = 0.2 + rand()*0.22;
                    leafH = 0.3 + rand()*0.26;

                    % The base sphere of the leaf
                    [sx,sy,sz] = sphere(baseRes);
                    [TH,PH] = cart2sph(sx,sy,sz);

                    % The noise of the leaf in the ellipsoid of the bush
                    noise1 = 0.05 * sin(3*TH).*cos(2*PH);
                    noise2 = 0.02 * randn(size(sx));
                    totalNoise = 1 + noise1 + noise2;
                    minN = 0.86;
                    maxN = 1.14;
                    totalNoise(totalNoise < minN) = minN;
                    totalNoise(totalNoise > maxN) = maxN;

                    sx = sx .* totalNoise;
                    sy = sy .* totalNoise;
                    sz = sz .* totalNoise;

                    % Change the shape of the leaf to be an ellipsoid
                    sx = sx * leafW;
                    sy = sy * leafW * (0.75 + rand()*0.35);
                    sz = sz * leafH;

                    % Random rotation of the leaf in the ellipsoid of the bush
                    rotZ = rand() * pi;
                    rotX = rand() * pi/4;
                    Rz = [cos(rotZ),-sin(rotZ),0;sin(rotZ),cos(rotZ),0;0,0,1];
                    Rx = [1,0,0;0,cos(rotX),-sin(rotX);0,sin(rotX),cos(rotX)];
                    rotMat = Rz * Rx;
                    allPts = rotMat * [sx(:)';sy(:)';sz(:)'];
                    sxRot = reshape(allPts(1,:), size(sx));
                    syRot = reshape(allPts(2,:), size(sy));
                    szRot = reshape(allPts(3,:), size(sz));

                    % The depth of bury the leaf in the ground
                    buryDepth = 0.15 + rand()*0.2;
                    szRot = szRot + buryDepth + lOffZ;

                    % The color of the leaf in the ellipsoid of the bush
                    heightRate = (lOffZ + bushRadiusZ) / (2*bushRadiusZ);
                    baseG = 0.18 + heightRate * 0.22;
                    rVal = baseG * (0.5 + rand()*0.3);
                    gVal = baseG + rand()*0.08;
                    bVal = baseG * (0.3 + rand()*0.25);
                    leafColor = [rVal, gVal, bVal];

                    finalX = sxRot + bx + lOffX;
                    finalY = syRot + by + lOffY;
                    finalZ = szRot + groundZ;

                    obj.surf_obj(finalX, finalY, finalZ, groundZ, leafColor, [0.82, 0.5, 0.6], 0.6 + rand() * 0.2);
                end

                % Store bush data for LiDAR detection
                obj.bushLocations = [obj.bushLocations; bx, by, bushRadiusX, bushRadiusZ, groundZ];

                % Mask the area around the bush
                distField = sqrt((obj.X - bx).^2 + (obj.Y - by).^2);
                maskRange = bushRadiusX + 0.6;
                obj.emptyLandMask(distField < maskRange) = 0;
                obj.validSlopeMask(distField < maskRange) = 0;
            end
        end

        function build_rock(obj, rock_number)

            if ~obj.renderRock
                return;
            end

            for i = 1:rock_number
                rx = rand() * 30;
                ry = rand() * 30;
                groundZ = obj.Fterrain(rx, ry);

                % rock size coefficient
                rcof = 1.2;
                % rock shape
                r_min = (0.4 + rand() * 0.4) * rcof;
                r_max = (r_min + rand() * 0.1) * rcof;
                h_rock = (0.4 + rand() * 0.2) * rcof;
                % disp([r_min, r_max, h_rock, groundZ]);

                obj.rockLocations = [obj.rockLocations; rx, ry, r_min, r_max, h_rock, groundZ];

                % rock orientation
                res = randi([30, 60]);
                [sx, sy, sz] = sphere(res);
                [TH, PH] = cart2sph(sx, sy, sz);

                noise1 = 0.25 * sin(2*TH) .* cos(1.5*PH);
                noise2 = 0.01 * randn(size(sx));

                totalNoise = 1 + noise1 + noise2;
                minR = 1.2;
                maxR = 1.8;
                totalNoise(totalNoise < minR) = minR;
                totalNoise(totalNoise > maxR) = maxR;

                sx = sx .* totalNoise;
                sy = sy .* totalNoise;
                sz = sz .* totalNoise;

                % Random scale rocks
                scaleX = r_max * (0.9 + rand()*0.25);
                scaleY = r_min * (0.9 + rand()*0.25);
                scaleZ = h_rock * (0.8 + rand()*0.4);
                sx = sx * scaleX;
                sy = sy * scaleY;
                sz = sz * scaleZ;

                % Random rotate rocks
                rotZ = rand() * 2*pi;
                rotX = rand() * pi/4;
                rotY = rand() * pi/5;

                Rz = [cos(rotZ), -sin(rotZ), 0;
                    sin(rotZ),  cos(rotZ), 0;
                    0,          0,         1];
                Rx = [1, 0,          0;
                    0, cos(rotX), -sin(rotX);
                    0, sin(rotX),  cos(rotX)];
                Ry = [cos(rotY), 0, sin(rotY);
                    0,         1, 0;
                    -sin(rotY), 0, cos(rotY)];
                rotMat = Rz * Ry * Rx;

                pts = rotMat * [sx(:)'; sy(:)'; sz(:)'];
                sx_rot = reshape(pts(1,:), size(sx));
                sy_rot = reshape(pts(2,:), size(sy));
                sz_rot = reshape(pts(3,:), size(sz));

                % Random bury rocks in the empty land
                buryRatio = 0.25 + rand()*0.3;
                sz_rot = sz_rot + buryRatio * h_rock;

                % The color of rocks
                rockColor = [0.45 + rand() * 0.1, 0.5 + rand() * 0.1, 0.55 + rand() * 0.1];
                obj.surf_obj(sx_rot + rx, sy_rot + ry, sz_rot + groundZ, groundZ, rockColor, ...
                    [rand()*0.5 + 0.1, rand()*0.5 + 0.1, rand()*0.5 + 0.1], 0.9);

                % Mask rocks in the empty land
                rockDist = sqrt((obj.X - rx).^2 + (obj.Y - ry).^2);
                maskRadius = r_max + 0.6;
                obj.emptyLandMask(rockDist < maskRadius) = 0;
                obj.validSlopeMask(rockDist < maskRadius) = 0;
            end

        end

        function build_fallen_trunk(obj, branchNum, posx, posy)
            disp('Generate fallen dead trunks with connected branches');
            if ~obj.renderBranch
                return;
            end

            baseRes = 20;
            branchRes = 12;

            % Get the ground height at the specified position
            rootGroundZ = obj.Fterrain(posx, posy);

            % Random trunk parameters
            trunkTotalLen = 4 + rand() * 2.8;
            trunkRootRadius = 0.22 + rand() * 0.32;
            bendAmplitude = 0.18 + rand() * 0.3;
            fallYaw = rand() * pi;       % Random yaw angle
            fallPitch = rand() * pi/3;   % Random pitch angle

            % Generate trunk grid points
            [thetaGrid, sGrid] = meshgrid(linspace(0, 2*pi, baseRes), linspace(0, trunkTotalLen, baseRes));
            % Random trunk bend
            centerCurveZ = bendAmplitude * sin(sGrid / trunkTotalLen * pi * 1.2);
            % Base cylinder coordinates of the trunk
            locX = trunkRootRadius .* cos(thetaGrid);
            locY = sGrid;
            locZ = trunkRootRadius .* sin(thetaGrid) + centerCurveZ;
            % Random bark noise
            barkNoise = 0.035 * sin(4*thetaGrid) .* cos(2*sGrid/trunkTotalLen*pi);
            locX = locX .* (1 + barkNoise);
            locZ = locZ .* (1 + barkNoise);

            % Global rotation matrix for the trunk and branches
            Rz = [cos(fallYaw), -sin(fallYaw), 0;
                sin(fallYaw),  cos(fallYaw), 0;
                0,             0,            1];
            Rx = [1, 0,              0;
                0, cos(fallPitch), -sin(fallPitch);
                0, sin(fallPitch),  cos(fallPitch)];
            globalRotMat = Rz * Rx;

            % Global transform trunk coordinates to world coordinates
            rawPts = globalRotMat * [locX(:)'; locY(:)'; locZ(:)'];
            trunkX = reshape(rawPts(1,:), size(locX)) + posx;
            trunkY = reshape(rawPts(2,:), size(locY)) + posy;
            trunkZ = reshape(rawPts(3,:), size(locZ)) + rootGroundZ;

            % Random trunk color and material
            woodBaseR = 0.32 + rand()*0.08;
            woodBaseG = 0.26 + rand()*0.07;
            woodBaseB = 0.21 + rand()*0.06;
            trunkColor = [woodBaseR, woodBaseG, woodBaseB];
            woodMatParam = [0.72, 0.02, 0.6];
            woodAlpha = 0.98;
            % Draw trunk (auto clipping at boundary/underground)
            obj.surf_obj(trunkX, trunkY, trunkZ, rootGroundZ, trunkColor, woodMatParam, woodAlpha);

            % Generate branches
            for brIdx = 1 : branchNum
                % Random branch origin on the trunk length
                branchS = trunkTotalLen * (0.25 + rand()*0.65);
                % Branch curve height (bend amplitude)
                curveAtBranch = bendAmplitude * sin(branchS / trunkTotalLen * pi * 1.2);
                % Branch local origin (in trunk local coordinate system)
                branchLocalOrigin = [0; branchS; curveAtBranch];
                % Branch world origin (after trunk rotation)
                branchWorldOrigin = globalRotMat * branchLocalOrigin;
                brOriginX = branchWorldOrigin(1) + posx;
                brOriginY = branchWorldOrigin(2) + posy;
                brOriginZ = branchWorldOrigin(3) + rootGroundZ;

                % Random branch parameters
                brLength = 0.4 + rand()*1.1;
                brRadius = trunkRootRadius * (0.22 + rand()*0.35);
                brBend = 0.08 + rand()*0.15;

                % Branch local cylinder grid
                [brTh, brS] = meshgrid(linspace(0, 2*pi, branchRes), linspace(0, brLength, branchRes));
                brCurveZ = brBend * sin(brS / brLength * pi);
                brLocX = brRadius .* cos(brTh);
                brLocY = brS;
                brLocZ = brRadius .* sin(brTh) + brCurveZ;

                % Random branch local rotation
                brYaw = rand() * 2*pi;
                brPitch = rand() * pi/2.5;
                brRz = [cos(brYaw),-sin(brYaw),0; sin(brYaw),cos(brYaw),0; 0,0,1];
                brRx = [1,0,0; 0,cos(brPitch),-sin(brPitch); 0,sin(brPitch),cos(brPitch)];
                brLocalRot = brRz * brRx;

                % Branch world coordinates
                brRawPts = brLocalRot * [brLocX(:)'; brLocY(:)'; brLocZ(:)'];
                brGlobalPts = globalRotMat * brRawPts;
                brX = reshape(brGlobalPts(1,:), size(brLocX)) + brOriginX;
                brY = reshape(brGlobalPts(2,:), size(brLocY)) + brOriginY;
                brZ = reshape(brGlobalPts(3,:), size(brLocZ)) + brOriginZ;

                % Random branch color
                brColor = [woodBaseR*0.92, woodBaseG*0.92, woodBaseB*0.92];
                % Draw branch (auto clipping at boundary/underground)
                obj.surf_obj(brX, brY, brZ, rootGroundZ, brColor, woodMatParam, woodAlpha);
            end
        end

        function surf_obj(obj, x, y, z, groundZ, color, objMaterial, alphaVal)

            % Filter out the objects that are out of the map
            mapMin = 0;
            mapMax = obj.fieldSize;

            outBorderMask = (x < mapMin) | (x > mapMax) | (y < mapMin) | (y > mapMax);
            x(outBorderMask) = NaN;
            y(outBorderMask) = NaN;
            z(outBorderMask) = NaN;

            maskUnderGround = z < groundZ;
            x(maskUnderGround) = NaN;
            y(maskUnderGround) = NaN;
            z(maskUnderGround) = NaN;

            % disp(color);

            surf(x, y, z, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', alphaVal);
            material(objMaterial);
        end
    end
end
