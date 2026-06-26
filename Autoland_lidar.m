classdef Autoland_lidar < handle
    properties
        beamRange = 25;
        hRes = 192;
        vRes = 144;
        hFOV = 90;
        vFOV = 120;
        downsampleFactor = 6;

        % Terrain Data properties
        terr_X, terr_Y, terr_Z
        has_terrain = false;
        F_terrain % Interpolant object for robust surface querying

        % Obstacle Data properties
        rockLocations = []
        bushLocations = []
        treeLocations = []

        % Map boundary
        mapMinX = 0; mapMaxX = 30;
        mapMinY = 0; mapMaxY = 30;
    end

    methods
        function obj = Autoland_lidar(hFOV, vFOV, maxRange, hRes, vRes, downsampleFactor, map)
            if nargin >= 1 && ~isempty(hFOV), obj.hFOV = hFOV; end
            if nargin >= 2 && ~isempty(vFOV), obj.vFOV = vFOV; end
            if nargin >= 3 && ~isempty(maxRange), obj.beamRange = min(max(maxRange, 0), 70); end
            if nargin >= 4 && ~isempty(hRes), obj.hRes = hRes; end
            if nargin >= 5 && ~isempty(vRes), obj.vRes = vRes; end
            if nargin >= 6 && ~isempty(downsampleFactor), obj.downsampleFactor = max(1, downsampleFactor); end
            if nargin >= 7 && ~isempty(map)
                obj.treeLocations = map.treeLocations;
                obj.rockLocations = map.rockLocations;
                obj.bushLocations = map.bushLocations;
            end

            % generate terrain interpolant
            % obj.F_terrain = map.Fterrain;
            obj.setTerrain(map.X, map.Y, map.Z_ground);

        end
        
        function setTerrain(obj, X, Y, Z)
            obj.terr_X = X; obj.terr_Y = Y; obj.terr_Z = Z;
            obj.F_terrain = griddedInterpolant(X', Y', Z', 'linear', 'none');
            obj.has_terrain = true;
            obj.mapMinX = min(X(:)); obj.mapMaxX = max(X(:));
            obj.mapMinY = min(Y(:)); obj.mapMaxY = max(Y(:));
        end

        function scanPoints = scan(obj, uavPosition, uavYaw)

            % Map boundary check
            if uavPosition(1) < obj.mapMinX || uavPosition(1) > obj.mapMaxX || ...
                    uavPosition(2) < obj.mapMinY || uavPosition(2) > obj.mapMaxY
                scanPoints = [];
                return;
            end

            % === X-axis spherical coordinates: theta=0°=forward, theta=120°=rear-down 60° ===
            % theta: polar angle from +X axis (forward), 0° to vFOV
            % phi: azimuth around X axis, -45° to +45° (right=+ when looking along +X)
            % In body frame (Z-up):
            %   dX = cos(theta)
            %   dY = sin(theta) * sin(phi)
            %   dZ = -sin(theta) * cos(phi)  (negative = downward)
            %
            % At theta=0°: all phi map to [1,0,0] = forward (no upward tilt)
            % At theta=90°: dX=0, dZ=-cos(phi) < 0 (always downward, no pole convergence)
            % At theta=120°: dX=-0.5, dZ=-0.866*cos(phi) (rear-down, no pole)
            
            theta_angles = deg2rad(linspace(0, obj.vFOV, obj.vRes));
            phi_angles = deg2rad(linspace(-obj.hFOV/2, obj.hFOV/2, obj.hRes));
            [Theta_mesh, Phi_mesh] = meshgrid(theta_angles, phi_angles);
            
            % Direction in body frame (before yaw rotation)
            dX_body = cos(Theta_mesh);
            dY_body = sin(Theta_mesh) .* sin(Phi_mesh);
            dZ_body = -sin(Theta_mesh) .* cos(Phi_mesh);
            
            % Rotate by yaw around Z axis
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
            
            % 2. Terrain intersection (ray-marching with 1.0m step)
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
            if ~isempty(obj.treeLocations)
                % get tree id
                % treeid = treeLocations(:,1);
                
                % Tree locations
                tx = obj.treeLocations(:,2);
                ty = obj.treeLocations(:,3);
                
                % Tree trunk and canopy parameters
                tr = obj.treeLocations(:,4);
                th = obj.treeLocations(:,5);
                
                % Tree trunk bottom height
                tb = obj.treeLocations(:,6);
                
                % Tree canopy radius
                cr = obj.treeLocations(:,7);
                
                % Get number of trees
                numTrees = length(tx);
                
                for i = 1:numTrees
                    % disp(['Calculate tree id: ', num2str(treeid(i))]);
                    
                    % Trunk cylinders
                    dx_r = dirs(:,1);
                    dy_r = dirs(:,2);

                    % Location of the drone, this is the lidar center
                    ox = uav_pos(1) - tx(i);
                    oy = uav_pos(2) - ty(i);
                    

                    A = dx_r.^2 + dy_r.^2;
                    B = 2 .* (ox.*dx_r + oy.*dy_r);
                    C_cyl = ox.^2 + oy.^2 - tr(i)^2;
                    delta_cyl = B.^2 - 4.*A.*C_cyl;
                    valid_cyl = delta_cyl >= 0;
                    
                    if any(valid_cyl)
                        t_cyl = (-B(valid_cyl) - sqrt(delta_cyl(valid_cyl))) ./ (2.*A(valid_cyl));
                        hit_z = uav_pos(3) + t_cyl .* dirs(valid_cyl, 3);
                        z_valid = (hit_z >= tb(i)) & (hit_z <= tb(i) + th(i) + 0.5);
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

                    % Check branches
                end
            end
            
            % 4. Rock intersection (bounding spheres)
            if ~isempty(obj.rockLocations)
                rx = obj.rockLocations(:,1); ry = obj.rockLocations(:,2);
                rr_min = obj.rockLocations(:,3); rr_max = obj.rockLocations(:,4);
                rh = obj.rockLocations(:,5); rb = obj.rockLocations(:,6);
                numRocks = length(rx);
                for i = 1:numRocks
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
                        
            % 6. Bush intersection (spheres)
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
            
            % 重新计算：min_ranges 被设为边界距离的点才是真正的边界点
            boundary_hits = abs(min_ranges - t_boundary) < 1e-3;
            valid_hits = (min_ranges < obj.beamRange) & (~boundary_hits);
            if any(valid_hits)
                fullCloud = uav_pos + dirs(valid_hits,:) .* min_ranges(valid_hits);
                fullCloud = fullCloud + randn(size(fullCloud))*0.02;
                scanPoints = fullCloud(1:obj.downsampleFactor:end, :);
            else
                scanPoints = [];
            end
        end
    end
end
