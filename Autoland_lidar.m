classdef Autoland_lidar < handle
    properties
        hFOV
        vFOV
        h_rays
        beamRange = 25;       
        hRes = 192;           
        vRes = 96;            
        downsampleFactor = 6; 
        showRays = false;     
        
        % Terrain Data properties
        terr_X, terr_Y, terr_Z
        has_terrain = false;
        F_terrain % Interpolant object for robust surface querying
    end
    
    methods
        % Instantiate the LiDAR system
        function obj = Autoland_lidar(hFOV, vFOV, maxRange, hRes, vRes, downsampleFactor, showRays)
            % Parameters
            %   hFOV: Horizontal Field of View in degrees
            %   vFOV: Vertical Field of View in degrees
            %   maxRange: Maximum range in meters
            %   hRes: Horizontal resolution in rays
            %   vRes: Vertical resolution in rays
            %   downsampleFactor: Downsample factor for visual rays
            %   showRays: Whether to show visual rays
            
            obj.hFOV = hFOV; obj.vFOV = vFOV;
            if nargin >= 3 && ~isempty(maxRange), obj.beamRange = min(max(maxRange, 0), 70); end
            if nargin >= 4 && ~isempty(hRes), obj.hRes = hRes; end
            if nargin >= 5 && ~isempty(vRes), obj.vRes = vRes; end
            if nargin >= 6 && ~isempty(downsampleFactor), obj.downsampleFactor = max(1, downsampleFactor); end
            if nargin >= 7 && ~isempty(showRays), obj.showRays = showRays; end
            
            % Initialize visual rays if required
            if obj.showRays
                [h_angles, ~] = meshgrid(linspace(-obj.hFOV/2, obj.hFOV/2, 5), linspace(-obj.vFOV/2, obj.vFOV/2, 3));
                obj.h_rays = cell(numel(h_angles), 1);
                for k = 1:numel(h_angles)
                    obj.h_rays{k} = plot3([0,0], [0,0], [0,0], 'y:', 'LineWidth', 1);
                end
            end
        end
        
        % Import external real terrain surface data
        function setTerrain(obj, X, Y, Z)
            obj.terr_X = X;
            obj.terr_Y = Y;
            obj.terr_Z = Z;
            % Use scatteredInterpolant to bypass strict NDGRID/MESHGRID formatting errors
            obj.F_terrain = scatteredInterpolant(X(:), Y(:), Z(:), 'linear', 'none');
            obj.has_terrain = true;
        end
        
        function scanPoints = getScanCloud(obj, uavPosition, uavYaw, treeLocations)
            % 1. Generate ray direction matrices
            h_angles = linspace(uavYaw - deg2rad(obj.hFOV/2), uavYaw + deg2rad(obj.hFOV/2), obj.hRes);
            v_angles = deg2rad(linspace(-obj.vFOV/2, obj.vFOV/2, obj.vRes));
            [H_mesh, V_mesh] = meshgrid(h_angles, v_angles);
            
            dX = cos(H_mesh) .* cos(V_mesh); dY = sin(H_mesh) .* cos(V_mesh); dZ = sin(V_mesh);
            dirs = [dX(:), dY(:), dZ(:)]; 
            numRays = size(dirs, 1);
            
            min_ranges = repmat(obj.beamRange, numRays, 1); 
            uav_pos = uavPosition(:)'; 
            
            % =======================================================
            % 2. True 3D Terrain Intersection (Vectorized Ray-Marching)
            % =======================================================
            if obj.has_terrain
                valid_g = dirs(:, 3) < -1e-4; % Only process rays pointing towards the ground
                idx_g = find(valid_g);
                
                if ~isempty(idx_g)
                    step_size = 0.5; % Ray marching step size: 0.5 meters
                    t_steps = 0:step_size:obj.beamRange; % 1 x N_steps
                    
                    % Vectorized calculation of 3D coordinates for all valid rays
                    ray_X = uav_pos(1) + dirs(idx_g, 1) * t_steps; % num_down x N_steps
                    ray_Y = uav_pos(2) + dirs(idx_g, 2) * t_steps;
                    ray_Z = uav_pos(3) + dirs(idx_g, 3) * t_steps;
                    
                    % Robust interpolation of terrain height at ray coordinates
                    terr_Z_sampled = obj.F_terrain(ray_X, ray_Y);
                    
                    % Handle out-of-bounds queries by assigning a very low height
                    terr_Z_sampled(isnan(terr_Z_sampled)) = -1000;
                    
                    % Collision detection: when ray Z drops below terrain Z
                    hit_mask = ray_Z < terr_Z_sampled;
                    [hit_found, hit_idx] = max(hit_mask, [], 2);
                    
                    % Filter valid collision points
                    actual_hits = hit_found > 0 & hit_idx > 1;
                    valid_k = find(actual_hits);
                    final_idx_g = idx_g(valid_k);
                    valid_hit_idx = hit_idx(valid_k);
                    
                    if ~isempty(valid_k)
                        % Linear traceback interpolation to find the exact collision distance (t)
                        lin_idx_curr = sub2ind(size(ray_Z), valid_k, valid_hit_idx);
                        lin_idx_prev = sub2ind(size(ray_Z), valid_k, valid_hit_idx - 1);
                        
                        z_ray1 = ray_Z(lin_idx_prev); z_terr1 = terr_Z_sampled(lin_idx_prev);
                        z_ray2 = ray_Z(lin_idx_curr); z_terr2 = terr_Z_sampled(lin_idx_curr);
                        
                        t1 = t_steps(valid_hit_idx - 1)';
                        t2 = t_steps(valid_hit_idx)';
                        
                        diff1 = z_ray1 - z_terr1;
                        diff2 = z_ray2 - z_terr2;
                        
                        t_exact = t1 + (t2 - t1) .* diff1 ./ (diff1 - diff2);
                        
                        % Update minimum distances
                        update_mask = t_exact < min_ranges(final_idx_g);
                        min_ranges(final_idx_g(update_mask)) = t_exact(update_mask);
                    end
                end
            end
            
            % =======================================================
            % 3. Trunk and Canopy Intersection (Strict Geometric Solvers)
            % =======================================================
            if ~isempty(treeLocations)
                tx = treeLocations(:,1); ty = treeLocations(:,2);
                tr = treeLocations(:,3); th = treeLocations(:,4); tb = treeLocations(:,5);
                numTrees = length(tx);
                
                for i = 1:numTrees
                    % --- A. Tree Trunks (Cylinders) ---
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
                    
                    % --- B. Tree Canopies (Spheres) ---
                    cz = tb(i) + th(i); 
                    cr = tr(i) * 3.5;   
                    
                    vx = uav_pos(1) - tx(i); vy = uav_pos(2) - ty(i); vz = uav_pos(3) - cz;
                    
                    B_sph = 2 .* (vx.*dirs(:,1) + vy.*dirs(:,2) + vz.*dirs(:,3));
                    C_sph = vx.^2 + vy.^2 + vz.^2 - cr^2;
                    
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
            
            % Output the final point cloud
            valid_hits = min_ranges < obj.beamRange;
            if any(valid_hits)
                fullCloud = uav_pos + dirs(valid_hits, :) .* min_ranges(valid_hits);
                scanPoints = fullCloud(1:obj.downsampleFactor:end, :);
            else
                scanPoints = [];
            end
        end
        
        function updateBeams(obj, uavPosition, uavYaw)
            if ~obj.showRays, return; end 
            ray_idx = 1;
            for r_h = linspace(-obj.hFOV/2, obj.hFOV/2, 5)
                for r_v = linspace(-obj.vFOV/2, obj.vFOV/2, 3)
                    total_yaw = uavYaw + deg2rad(r_h);
                    dir = [cos(total_yaw)*cosd(r_v); sin(total_yaw)*cosd(r_v); sind(r_v)];
                    ray_end = uavPosition + dir * obj.beamRange;
                    set(obj.h_rays{ray_idx}, 'XData', [uavPosition(1), ray_end(1)], ...
                                         'YData', [uavPosition(2), ray_end(2)], ...
                                         'ZData', [uavPosition(3), ray_end(3)]);
                    ray_idx = ray_idx + 1;
                end
            end
        end
    end
end