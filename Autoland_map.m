classdef Autoland_map < handle
    properties
        % Map parameters
        mapSize = 30;       % Map size in meters
        gridRes = 0.5;      % Grid resolution in meters
        X, Y, Z_ground;     % Terrain grid
        Fterrain;           % Terrain height interpolation function

        % Obstacle coordinates storage matrix
        treeLocations = []; % [tx, ty, trunkR, trunkH, h_base, canopyR, branchNum]
        rockLocations = []; % Rock [rx, ry, r_min, r_max, h_rock, h_base]
        bushLocations = []; % Bush grass [bx, by, r_bush, h_bush, h_base]
        leafClusters = [];  % Leaf clusters [cx, cy, cz, radius] from branch ends

        %% Render handles, for visibility control
        visible = true;     % Map visibility flag
        h_terrain
        h_trees = []
        h_rocks = []
        h_bushes = []
        h_trunks = []
    end

    methods
        function obj = Autoland_map(mapSize, gridRes)
            obj.mapSize = mapSize;
            obj.gridRes = gridRes;

            disp('Building basic terrain grid...');
            % Generate basic terrain grid
            [obj.X, obj.Y] = meshgrid(0:obj.gridRes:obj.mapSize);

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

            % Create terrain height interpolation function
            obj.Fterrain = scatteredInterpolant(obj.X(:), obj.Y(:), obj.Z_ground(:), 'linear', 'nearest');

            % Generate environment
            obj.generateEnvironment();
        end

        % Main environment generator function, step-by-step execution
        function generateEnvironment(obj)
            rng(42); % Fix random seed for consistent environment generation
            hold on; grid off;

            % Render basic terrain grass
            disp('Render basic terrain grass');
            obj.h_terrain = surf(obj.X, obj.Y, obj.Z_ground, 'EdgeColor', 'none', 'FaceAlpha', 0.9);

            % Set colormap to summer for better visibility
            colormap(summer);
            light;
            lighting gouraud;

            % Render trees
            disp('Render trees');
            numTrees = randi([8,12]); % Random 8-12 trees
            for i = 1:numTrees
                % Make sure trees are not too close to the edges
                tx = 5 + rand() * 20;
                ty = 5 + rand() * 20;
                h_base = obj.Fterrain(tx, ty); % Tree base height

                % Tree trunk parameters
                trunkR = 0.4 + rand() * 0.4;
                trunkH = 4 + rand() * 4;

                % Canopy radius of the tree
                canopyR = 1.8 + rand() * 1.2;

                % Branch number per tree
                branchNum = randi([4,7]);

                % Store tree data
                obj.treeLocations = [obj.treeLocations; i, tx, ty, trunkR, trunkH, h_base, canopyR];

                % Draw tree trunk cylinder
                [cX,cY,cZ] = cylinder(trunkR,16);
                trunk = surf(cX+tx, cY+ty, cZ*trunkH + h_base, ...
                    'FaceColor', [0.42,0.24,0.06], 'EdgeColor','none');

                % save trunk handle for visible control
                obj.h_trees = [obj.h_trees; trunk];

                % Draw tree canopy top
                [sx,sy,sz] = sphere(20);
                canopy = surf(sx*canopyR + tx, sy*canopyR + ty, sz*canopyR + h_base + trunkH, ...
                    'FaceColor', [0.12,0.52,0.15], 'EdgeColor','none','FaceAlpha',0.5);

                % save canopy handle for visible control
                obj.h_trees = [obj.h_trees; canopy];

                % Draw tree branches
                for b = 1:branchNum
                    % Tree branch parameters
                    branchLen = 1.2 + rand()*1.0;           % Branch length
                    branchR = trunkR * (0.2 + rand()*0.3);  % Branch radius
                    branchAngleX = rand()*2*pi;             % Branch angle X, around trunk direction
                    branchAngleZ = pi/4 + rand()*pi/3;      % Branch angle Z, diagonally upward
                    branchBaseZ = h_base + trunkH * (0.4 + rand()*0.5); % Branch base height

                    % Transform branch coordinates to tree trunk
                    [brX, brY, brZ] = cylinder(branchR, 8);

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

                    branch = surf(brX_rot + tx, brY_rot + ty, brZ_rot + branchBaseZ, ...
                        'FaceColor', [0.38,0.21,0.04], 'EdgeColor','none');

                    % save branch handle for visible control
                    obj.h_trees = [obj.h_trees; branch];

                    % Draw leaf cluster at branch end
                    leafR = canopyR * (0.4 + rand()*0.6);
                    leafCx = tx + brX_rot(end,end);
                    leafCy = ty + brY_rot(end,end);
                    leafCz = branchBaseZ + brZ_rot(end,end);
                    [sx,sy,sz] = sphere(12);

                    % Draw leaf cluster at branch end
                    leaf = surf(sx*leafR + leafCx, ...
                        sy*leafR + leafCy, ...
                        sz*leafR + leafCz, ...
                        'FaceColor', [0.08,0.45,0.12], 'EdgeColor','none','FaceAlpha',0.6);

                    % save leaf handle for visible control
                    obj.h_trees = [obj.h_trees; leaf];

                    % Store leaf cluster for LiDAR detection
                    obj.leafClusters = [obj.leafClusters; leafCx, leafCy, leafCz, leafR];
                end
            end

            disp('Render bush grass');
            obj.build_bushgrass(randi([12,18]));

            disp('Render rocks');
            obj.build_rock(randi([8, 15]));

            disp('Render fallen dead trunks with branches');
            obj.build_fallen_trunk(10, 15, 20);

            %% Step9: Draw coordinate axis and view angle
            axis equal;

            disp('Draw coordinate axis and view angle');
            xlabel('X (m)'); ylabel('Y (m)');
            zlabel('Altitude (m)');
            xlim([0 obj.mapSize]);
            ylim([0 obj.mapSize]);
            zlim([0 25])

            % fix axis
            axis manual;
            axis vis3d;

            % view(45, 30);
            pos = get(gcf, 'Position');
            set(gcf, 'Position', [pos(1) pos(2) 1024 768]);

            hold off;
            disp('Map environment generated successfully.');
        end

        function visTog(obj)
            obj.visible = ~obj.visible;
            set(obj.h_terrain, 'Visible', obj.visible);
            set(obj.h_trees, 'Visible', obj.visible);
            set(obj.h_rocks, 'Visible', obj.visible);
            set(obj.h_bushes, 'Visible', obj.visible);
            set(obj.h_trunks, 'Visible', obj.visible);
        end

    end
    methods (Access = private)
        % Private methods

        function build_bushgrass(obj, bush_number)
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
                    leafL = leafW * (0.75 + rand()*0.35);
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
                    sy = sy * leafL;
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

                    % Record the leaf center and size
                    leafCenterX = bx + lOffX;
                    leafCenterY = by + lOffY;
                    leafGroundZ = groundZ + lOffZ;

                    % Save each leaf center and size in the bushLocations
                    obj.bushLocations = [obj.bushLocations; leafCenterX, leafCenterY, leafW, leafL, leafH, leafGroundZ];

                    % Surf the leaf
                    blf_hd = obj.surf_obj(finalX, finalY, finalZ, groundZ, leafColor, [0.82, 0.5, 0.6], 0.6 + rand() * 0.2);
                    obj.h_bushes = [obj.h_bushes, blf_hd];
                end

            end
        end

        function build_rock(obj, rock_number)
            for i = 1:rock_number
                % Random position of the rock
                rx = rand() * 30;
                ry = rand() * 30;
                groundZ = obj.Fterrain(rx, ry);

                rcof = 1.2;
                r_min = (0.4 + rand() * 0.4) * rcof;
                r_max = (r_min + rand() * 0.1) * rcof;
                h_rock = (0.4 + rand() * 0.2) * rcof;

                % The axis of rocks
                scaleX = r_max * (0.9 + rand()*0.25);
                scaleY = r_min * (0.9 + rand()*0.25);
                scaleZ = h_rock * (0.8 + rand()*0.4);

                % Random rotation of the rock
                rotZ = rand() * 2*pi;
                rotX = rand() * pi/4;
                rotY = rand() * pi/5;

                % The depth of bury the rock in the ground
                buryRatio = 0.25 + rand()*0.3;
                buryDepth = buryRatio * h_rock;
                cz = groundZ + buryDepth;

                % Save each rock center and size in the rockLocations
                obj.rockLocations = [obj.rockLocations; rx, ry, cz, scaleX, scaleY, scaleZ, rotZ, rotX, rotY];

                % The rotation matrix of the rock
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

                % Generate the rock surface
                res = randi([30, 60]);
                [sx, sy, sz] = sphere(res);
                [TH, PH] = cart2sph(sx, sy, sz);

                noise1 = 0.25 * sin(2*TH) .* cos(1.5*PH);
                noise2 = 0.01 * randn(size(sx));
                totalNoise = 1 + noise1 + noise2;
                totalNoise(totalNoise < 1.2) = 1.2;
                totalNoise(totalNoise > 1.8) = 1.8;

                sx = sx .* totalNoise * scaleX;
                sy = sy .* totalNoise * scaleY;
                sz = sz .* totalNoise * scaleZ;

                pts = rotMat * [sx(:)'; sy(:)'; sz(:)'];
                sx_rot = reshape(pts(1,:), size(sx));
                sy_rot = reshape(pts(2,:), size(sy));
                sz_rot = reshape(pts(3,:), size(sz));

                sz_rot = sz_rot + buryDepth;

                rockColor = [0.45 + rand()*0.1, 0.5 + rand()*0.1, 0.55 + rand()*0.1];
                rock_hd = obj.surf_obj(sx_rot + rx, sy_rot + ry, sz_rot + groundZ, groundZ, rockColor, ...
                    [rand()*0.5 + 0.1, rand()*0.5 + 0.1, rand()*0.5 + 0.1], 0.9);

                obj.h_rocks = [obj.h_rocks, rock_hd];
            end
        end

        function build_fallen_trunk(obj, branchNum, posx, posy)
            disp('Generate fallen dead trunks with connected branches');
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
            fall_trunk_hd = obj.surf_obj(trunkX, trunkY, trunkZ, rootGroundZ, trunkColor, woodMatParam, woodAlpha);
            obj.h_trunks = [obj.h_trunks, fall_trunk_hd];

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
                fall_br_hd = obj.surf_obj(brX, brY, brZ, rootGroundZ, brColor, woodMatParam, woodAlpha);
                obj.h_trunks = [obj.h_trunks, fall_br_hd];
            end
        end

        function hd = surf_obj(obj, x, y, z, groundZ, color, objMaterial, alphaVal)

            % Filter out the objects that are out of the map
            mapMin = 0;
            mapMax = obj.mapSize;

            outBorderMask = (x < mapMin) | (x > mapMax) | (y < mapMin) | (y > mapMax);
            x(outBorderMask) = NaN;
            y(outBorderMask) = NaN;
            z(outBorderMask) = NaN;

            maskUnderGround = z < groundZ;
            x(maskUnderGround) = NaN;
            y(maskUnderGround) = NaN;
            z(maskUnderGround) = NaN;

            % disp(color);

            hd = surf(x, y, z, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', alphaVal);
            material(objMaterial);
        end
    end
end
