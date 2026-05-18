classdef Autoland_lidar < handle
    properties
        hFOV
        vFOV
        h_rays
        beamRange = 15; % Maximum telemetry tracing distance limit
    end
    
    methods
        function obj = Autoland_lidar(hFOV, vFOV)
            obj.hFOV = hFOV;
            obj.vFOV = vFOV;
            
            % Derive directional scanning matrix combinations
            [h_angles, ~] = meshgrid(linspace(-obj.hFOV/2, obj.hFOV/2, 5), linspace(-obj.vFOV/2, obj.vFOV/2, 3));
            % [h_angles, v_angles] = meshgrid(linspace(-obj.hFOV/2, obj.hFOV/2, 5), linspace(-obj.vFOV/2, obj.vFOV/2, 3));
            obj.h_rays = cell(numel(h_angles), 1);
            
            % Instantiating distinct handle representations
            for k = 1:numel(h_angles)
                obj.h_rays{k} = plot3([0,0], [0,0], [0,0], 'y:', 'LineWidth', 1);
            end
        end
        
        function updateBeams(obj, uavPosition, uavYaw)
            ray_idx = 1;
            for r_h = linspace(-obj.hFOV/2, obj.hFOV/2, 5)
                for r_v = linspace(-obj.vFOV/2, obj.vFOV/2, 3)
                    
                    % Map relative orientation attributes against global coordinate frameworks
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