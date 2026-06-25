classdef Autoland_lidar < handle
    properties
        beamRange = 25;
        hRes = 192;
        vRes = 144;
        hFOV = 90;
        vFOV = 120;
        downsampleFactor = 6;
        showRays = false;
        h_rays

        % Terrain Data properties
        terr_X, terr_Y, terr_Z
        has_terrain = false;
        F_terrain % Interpolant object for robust surface querying

        % Obstacle Data properties
        rockLocations = []
        stumpPos = []
        bushLocations = []

        % Map boundary
        mapMinX = 0; mapMaxX = 30;
        mapMinY = 0; mapMaxY = 30;
    end

    methods
        function obj = Autoland_lidar(hFOV, vFOV, maxRange, hRes, vRes, downsampleFactor, showRays)
            if nargin >= 1 && ~isempty(hFOV), obj.hFOV = hFOV; end
            if nargin >= 2 && ~isempty(vFOV), obj.vFOV = vFOV; end
            if nargin >= 3 && ~isempty(maxRange), obj.beamRange = min(max(maxRange, 0), 70); end
            if nargin >= 4 && ~isempty(hRes), obj.hRes = hRes; end
            if nargin >= 5 && ~isempty(vRes), obj.vRes = vRes; end
            if nargin >= 6 && ~isempty(downsampleFactor), obj.downsampleFactor = max(1, downsampleFactor); end
            if nargin >= 7 && ~isempty(showRays), obj.showRays = showRays; end

            if obj.showRays
                [h_angles, ~] = meshgrid(linspace(-obj.hFOV/2, obj.hFOV/2, 5) - 90, linspace(0, -obj.vFOV, 3));
                obj.h_rays = cell(numel(h_angles), 1);
                for k = 1:numel(h_angles)
                    obj.h_rays{k} = plot3([0,0], [0,0], [0,0], 'y:', 'LineWidth', 1);
                end
            end
        end

        function setTerrain(obj, X, Y, Z)
            obj.terr_X = X; obj.terr_Y = Y; obj.terr_Z = Z;
            obj.F_terrain = griddedInterpolant(X', Y', Z', 'linear', 'none');
            obj.has_terrain = true;
            obj.mapMinX = min(X(:)); obj.mapMaxX = max(X(:));
            obj.mapMinY = min(Y(:)); obj.mapMaxY = max(Y(:));
        end

        function setObstacles(obj, rockLocations, stumpPos, bushLocations)
            obj.rockLocations = rockLocations;
            obj.stumpPos = stumpPos;
            obj.bushLocations = bushLocations;
        end

        function scanPoints = getScanCloud(obj, uavPosition, uavYaw, treeLocations)
            % Only scan when inside map
            if uavPosition(1) < obj.mapMinX || uavPosition(1) > obj.mapMaxX || ...
                    uavPosition(2) < obj.mapMinY || uavPosition(2) > obj.mapMaxY
                scanPoints = [];
                return;
            end

            % Generate pitch (elevation) and roll (azimuth) angles
            % Pitch: 0 deg (horizontal forward) to -vFOV deg (rear-down)
            % Roll: centered at -90 deg (forward), span hFOV deg

            % pitch_angles = deg2rad(linspace(0, -obj.vFOV, obj.vRes));

            % roll_angles  = deg2rad(linspace(-obj.hFOV/2, obj.hFOV/2, obj.hRes) - 90);

            % [Pitch_mesh, Roll_mesh] = meshgrid(pitch_angles, roll_angles);

            % % Convert user roll definition to standard azimuth (0 = forward)
            % azimuth = Roll_mesh + deg2rad(90);
            % abs_pitch = abs(Pitch_mesh);

            % % Direction in body frame (Z-up), then rotate by yaw
            % dX_body = cos(abs_pitch) .* cos(azimuth);
            % dY_body = cos(abs_pitch) .* sin(azimuth);
            % dZ_body = -sin(abs_pitch);

            % dX = dX_body .* cos(uavYaw) - dY_body .* sin(uavYaw);
            % dY = dX_body .* sin(uavYaw) + dY_body .* cos(uavYaw);
            % dZ = dZ_body;

            % 垂直角度：从水平(0°)向下扫到 -vFOV
            % 映射到 phi：0° → pi/2，-vFOV → pi/2 + |vFOV|
            pitch_angles = deg2rad(linspace(0, -obj.vFOV, obj.vRes));

            % 水平角度：直接就是方位角 theta，0 = 正前方
            roll_angles = deg2rad(linspace(-obj.hFOV/2, obj.hFOV/2, obj.hRes));

            [Pitch_mesh, Roll_mesh] = meshgrid(pitch_angles, roll_angles);

            %% 关键转换：pitch → phi，让中心线(0°) = 赤道(pi/2)
            % pitch = 0°  → phi = pi/2（水平向前，赤道）
            % pitch < 0°  → phi > pi/2（向下，南半球）
            phi = pi/2 - Pitch_mesh;        % 0°→pi/2, -90°→pi

            % 方位角直接用
            theta = Roll_mesh;

            %% 标准ISO球坐标 → 直角坐标（Z-up，X-前，Y-右）
            dX_body = sin(phi) .* cos(theta);
            dY_body = sin(phi) .* sin(theta);
            dZ_body = cos(phi);             % 水平=0，向下<0，和原代码一致

            %% 偏航旋转（不变）
            dX = dX_body .* cos(uavYaw) - dY_body .* sin(uavYaw);
            dY = dX_body .* sin(uavYaw) + dY_body .* cos(uavYaw);
            dZ = dZ_body;

            dirs = [dX(:), dY(:), dZ(:)];
            numRays = size(dirs, 1);
            min_ranges = repmat(obj.beamRange, numRays, 1);
            uav_pos = uavPosition(:)';

            % 1. Map boundary clipping
            dx = dirs(:, 1); dy = dirs(:, 2);
            t_x = inf(numRays, 1);
            t_x(dx > 1e-9) = (obj.mapMaxX - uav_pos(1)) ./ dx(dx > 1e-9);
            t_x(dx < -1e-9) = (obj.mapMinX - uav_pos(1)) ./ dx(dx < -1e-9);
            t_x(t_x <= 0) = inf;

            t_y = inf(numRays, 1);
            t_y(dy > 1e-9) = (obj.mapMaxY - uav_pos(2)) ./ dy(dy > 1e-9);
            t_y(dy < -1e-9) = (obj.mapMinY - uav_pos(2)) ./ dy(dy < -1e-9);
            t_y(t_y <= 0) = inf;

            t_boundary = min(t_x, t_y);
            boundary_hits = t_boundary < min_ranges;
            min_ranges(boundary_hits) = t_boundary(boundary_hits);

            % 2. Terrain intersection (ray-marching, only downward rays)
            if obj.has_terrain
                valid_g = dirs(:, 3) < -1e-4;
                idx_g = find(valid_g);
                if ~isempty(idx_g)
                    step_size = 1.0;
                    t_steps = 0:step_size:obj.beamRange;

                    ray_X = uav_pos(1) + dirs(idx_g, 1) * t_steps;
                    ray_Y = uav_pos(2) + dirs(idx_g, 2) * t_steps;
                    ray_Z = uav_pos(3) + dirs(idx_g, 3) * t_steps;

                    terr_Z_sampled = obj.F_terrain(ray_X, ray_Y);
                    terr_Z_sampled(isnan(terr_Z_sampled)) = -1000;

                    hit_mask = ray_Z < terr_Z_sampled;
                    [hit_found, hit_idx] = max(hit_mask, [], 2);
                    actual_hits = hit_found > 0 & hit_idx > 1;
                    valid_k = find(actual_hits);
                    final_idx_g = idx_g(valid_k);
                    valid_hit_idx = hit_idx(valid_k);

                    if ~isempty(valid_k)
                        lin_idx_curr = sub2ind(size(ray_Z), valid_k, valid_hit_idx);
                        lin_idx_prev = sub2ind(size(ray_Z), valid_k, valid_hit_idx - 1);

                        z_ray1 = ray_Z(lin_idx_prev); z_terr1 = terr_Z_sampled(lin_idx_prev);
                        z_ray2 = ray_Z(lin_idx_curr); z_terr2 = terr_Z_sampled(lin_idx_curr);

                        t1 = t_steps(valid_hit_idx - 1)';
                        t2 = t_steps(valid_hit_idx)';

                        diff1 = z_ray1 - z_terr1;
                        diff2 = z_ray2 - z_terr2;
                        t_exact = t1 + (t2 - t1) .* diff1 ./ (diff1 - diff2);

                        update_mask = t_exact < min_ranges(final_idx_g);
                        min_ranges(final_idx_g(update_mask)) = t_exact(update_mask);
                    end
                end
            end

            % 3. Tree trunk and canopy intersection
            if ~isempty(treeLocations)
                tx = treeLocations(:,1); ty = treeLocations(:,2);
                tr = treeLocations(:,3); th = treeLocations(:,4); tb = treeLocations(:,5); cr = treeLocations(:,6);
                numTrees = length(tx);
                for i = 1:numTrees
                    % Trunk cylinders
                    dx = dirs(:,1); dy = dirs(:,2);
                    ox = uav_pos(1) - tx(i); oy = uav_pos(2) - ty(i);
                    A = dx.^2 + dy.^2; B = 2 .* (ox.*dx + oy.*dy); C_cyl = ox.^2 + oy.^2 - tr(i)^2;
                    delta_cyl = B.^2 - 4.*A.*C_cyl;
                    valid_cyl = delta_cyl >= 0;
                    if any(valid_cyl)
                        t_cyl = (-B(valid_cyl) - sqrt(delta_cyl(valid_cyl))) ./ (2.*A(valid_cyl));
                        hit_z = uav_pos(3) + t_cyl .* dirs(valid_cyl, 3);
                        z_valid = (hit_z >= tb(i)) & (hit_z <= tb(i) + th(i)*0.5);
                        idx = find(valid_cyl);
                        final_cyl = idx(t_cyl > 0 & t_cyl < min_ranges(idx) & z_valid);
                        min_ranges(final_cyl) = t_cyl(t_cyl > 0 & t_cyl < min_ranges(idx) & z_valid);
                    end
                    % Canopy spheres
                    cz = tb(i) + th(i);
                    vx = uav_pos(1) - tx(i); vy = uav_pos(2) - ty(i); vz = uav_pos(3) - cz;
                    B_sph = 2 .* (vx.*dirs(:,1) + vy.*dirs(:,2) + vz.*dirs(:,3));
                    C_sph = vx.^2 + vy.^2 + vz.^2 - cr(i)^2;
                    delta_sph = B_sph.^2 - 4.*C_sph;
                    valid_sph = delta_sph >= 0;
                    if any(valid_sph)
                        t_sph = (-B_sph(valid_sph) - sqrt(delta_sph(valid_sph))) ./ 2;
                        idx_sph = find(valid_sph);
                        final_sph = idx_sph(t_sph > 0 & t_sph < min_ranges(idx_sph));
                        min_ranges(final_sph) = t_sph(t_sph > 0 & t_sph < min_ranges(idx_sph));
                    end
                end
            end

            % 4. Rock intersection (approximate as ellipsoids/bounding spheres)
            if ~isempty(obj.rockLocations)
                rx = obj.rockLocations(:,1); ry = obj.rockLocations(:,2);
                rr_min = obj.rockLocations(:,3); rr_max = obj.rockLocations(:,4);
                rh = obj.rockLocations(:,5); rb = obj.rockLocations(:,6);
                numRocks = length(rx);
                for i = 1:numRocks
                    % Bounding sphere: radius = max of the three semi-axes
                    rockR = max([rr_min(i), rr_max(i), rh(i)]) * 0.6;
                    rockCx = rx(i); rockCy = ry(i); rockCz = rb(i) + rh(i)*0.5;
                    vx = uav_pos(1) - rockCx; vy = uav_pos(2) - rockCy; vz = uav_pos(3) - rockCz;
                    B_sph = 2 .* (vx.*dirs(:,1) + vy.*dirs(:,2) + vz.*dirs(:,3));
                    C_sph = vx.^2 + vy.^2 + vz.^2 - rockR^2;
                    delta_sph = B_sph.^2 - 4.*C_sph;
                    valid_sph = delta_sph >= 0;
                    if any(valid_sph)
                        t_sph = (-B_sph(valid_sph) - sqrt(delta_sph(valid_sph))) ./ 2;
                        idx_sph = find(valid_sph);
                        final_sph = idx_sph(t_sph > 0 & t_sph < min_ranges(idx_sph));
                        min_ranges(final_sph) = t_sph(t_sph > 0 & t_sph < min_ranges(idx_sph));
                    end
                end
            end

            % 5. Stump intersection (cylinders, similar to tree trunks)
            if ~isempty(obj.stumpPos)
                sx = obj.stumpPos(:,1); sy = obj.stumpPos(:,2);
                sr = obj.stumpPos(:,3); sh = obj.stumpPos(:,4); sb = obj.stumpPos(:,5);
                numStumps = length(sx);
                for i = 1:numStumps
                    dx = dirs(:,1); dy = dirs(:,2);
                    ox = uav_pos(1) - sx(i); oy = uav_pos(2) - sy(i);
                    A = dx.^2 + dy.^2; B = 2 .* (ox.*dx + oy.*dy); C_cyl = ox.^2 + oy.^2 - sr(i)^2;
                    delta_cyl = B.^2 - 4.*A.*C_cyl;
                    valid_cyl = delta_cyl >= 0;
                    if any(valid_cyl)
                        t_cyl = (-B(valid_cyl) - sqrt(delta_cyl(valid_cyl))) ./ (2.*A(valid_cyl));
                        hit_z = uav_pos(3) + t_cyl .* dirs(valid_cyl, 3);
                        z_valid = (hit_z >= sb(i)) & (hit_z <= sb(i) + sh(i));
                        idx = find(valid_cyl);
                        final_cyl = idx(t_cyl > 0 & t_cyl < min_ranges(idx) & z_valid);
                        min_ranges(final_cyl) = t_cyl(t_cyl > 0 & t_cyl < min_ranges(idx) & z_valid);
                    end
                end
            end

            % 6. Bush intersection (approximate as spheres)
            if ~isempty(obj.bushLocations)
                bx = obj.bushLocations(:,1); by = obj.bushLocations(:,2);
                br = obj.bushLocations(:,3); bh = obj.bushLocations(:,4); bb = obj.bushLocations(:,5);
                numBushes = length(bx);
                for i = 1:numBushes
                    bushR = br(i);
                    bushCx = bx(i); bushCy = by(i); bushCz = bb(i) + bh(i)*0.5;
                    vx = uav_pos(1) - bushCx; vy = uav_pos(2) - bushCy; vz = uav_pos(3) - bushCz;
                    B_sph = 2 .* (vx.*dirs(:,1) + vy.*dirs(:,2) + vz.*dirs(:,3));
                    C_sph = vx.^2 + vy.^2 + vz.^2 - bushR^2;
                    delta_sph = B_sph.^2 - 4.*C_sph;
                    valid_sph = delta_sph >= 0;
                    if any(valid_sph)
                        t_sph = (-B_sph(valid_sph) - sqrt(delta_sph(valid_sph))) ./ 2;
                        idx_sph = find(valid_sph);
                        final_sph = idx_sph(t_sph > 0 & t_sph < min_ranges(idx_sph));
                        min_ranges(final_sph) = t_sph(t_sph > 0 & t_sph < min_ranges(idx_sph));
                    end
                end
            end

            valid_hits = min_ranges < obj.beamRange;
            if any(valid_hits)
                fullCloud = uav_pos + dirs(valid_hits,:) .* min_ranges(valid_hits);
                fullCloud = fullCloud + randn(size(fullCloud))*0.02;
                scanPoints = fullCloud(1:obj.downsampleFactor:end, :);
            else
                scanPoints = [];
            end
        end

        function updateBeams(obj, uavPosition, uavYaw)
            if ~obj.showRays, return; end
            ray_idx = 1;
            for r_roll = linspace(-obj.hFOV/2, obj.hFOV/2, 5) - 90
                for r_pitch = linspace(0, -obj.vFOV, 3)
                    az = deg2rad(r_roll + 90);
                    abs_p = abs(deg2rad(r_pitch));
                    dXb = cos(abs_p) * cos(az);
                    dYb = cos(abs_p) * sin(az);
                    dZb = -sin(abs_p);
                    dir = [dXb*cos(uavYaw) - dYb*sin(uavYaw);
                        dXb*sin(uavYaw) + dYb*cos(uavYaw);
                        dZb];
                    t_max = obj.beamRange;
                    if dir(1) > 1e-9
                        t_bound = (obj.mapMaxX - uavPosition(1)) / dir(1);
                    elseif dir(1) < -1e-9
                        t_bound = (obj.mapMinX - uavPosition(1)) / dir(1);
                    else
                        t_bound = inf;
                    end

                    if t_bound > 0 && t_bound < t_max
                        t_max = t_bound;
                    end

                    if dir(2) > 1e-9
                        t_bound = (obj.mapMaxY - uavPosition(2)) / dir(2);
                    elseif dir(2) < -1e-9
                        t_bound = (obj.mapMinY - uavPosition(2)) / dir(2);
                    else
                        t_bound = inf;
                    end

                    if t_bound > 0 && t_bound < t_max
                        t_max = t_bound;
                    end

                    ray_end = uavPosition + dir * t_max;

                    set(obj.h_rays{ray_idx}, 'XData', [uavPosition(1), ray_end(1)], ...
                        'YData', [uavPosition(2), ray_end(2)], 'ZData', [uavPosition(3), ray_end(3)]);
                    ray_idx = ray_idx + 1;
                end
            end
        end
    end
end
