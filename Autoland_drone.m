classdef Autoland_drone < handle
    properties
        Position = [5; 5; 10]; % Initial position [x; y; z]
        Velocity = [0; 0; 0]; % Initial velocity [vx; vy; vz]
        Yaw = 0;              % Yaw angle (rad)
        MaxSpeed = 5;         % Maximum speed (m/s)
        dt = 0.05;            % Time delta step interval
        Kp = 1.5;             % Direct navigation tracking proportional gain
        
        % Artificial Potential Field configuration constants
        K_att = 0.4;          % Attractor scaling gain coefficient
        K_rep = 30.0;         % Repelling obstacle gain coefficient
        influence_dist = 7.0; % Proximity metric defining radar avoidance boundary
        
        % Component Graphic Handles
        h_arm1, h_arm2
        h_prop1, h_prop2, h_prop3, h_prop4
        h_path, uav_history
        armLength = 1.2;
        propRadius = 0.4;
        cx, cy, cz
    end
    
    methods
        function obj = Autoland_drone(startPos)
            obj.Position = startPos(:);
            obj.uav_history = obj.Position;
            obj.initGraphics();
        end
        
        function initGraphics(obj)
            hold on;
            % Build structure framework
            obj.h_arm1 = plot3([0,0], [0,0], [0,0], 'Color', [0.2 0.2 0.2], 'LineWidth', 3);
            obj.h_arm2 = plot3([0,0], [0,0], [0,0], 'Color', [0.2 0.2 0.2], 'LineWidth', 3);
            
            % Parametrizing rotors circumferential trace outlines
            theta_circle = linspace(0, 2*pi, 12);
            obj.cx = obj.propRadius * cos(theta_circle);
            obj.cy = obj.propRadius * sin(theta_circle);
            obj.cz = zeros(size(theta_circle));
            
            % Generate patch surface geometries matching quadcopter convention templates
            obj.h_prop1 = fill3(obj.cx, obj.cy, obj.cz, 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); 
            obj.h_prop2 = fill3(obj.cx, obj.cy, obj.cz, 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); 
            obj.h_prop3 = fill3(obj.cx, obj.cy, obj.cz, 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); 
            obj.h_prop4 = fill3(obj.cx, obj.cy, obj.cz, 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); 
            
            obj.h_path = plot3(obj.Position(1), obj.Position(2), obj.Position(3), 'b-', 'LineWidth', 1.5);
        end
        
        function update(obj, globalTargetPos, treeLocations)
            % 1. Extract Attraction Force vector properties
            F_att = obj.K_att * (globalTargetPos(:) - obj.Position);
            
            % 2. Parse repulsion vectors mapping current tree locations matrix indices
            F_rep = [0; 0; 0];
            for i = 1:size(treeLocations, 1)
                tx = treeLocations(i,1); ty = treeLocations(i,2);
                tr = treeLocations(i,3); th = treeLocations(i,4); tb = treeLocations(i,5);
                
                % Compute horizontal Euclidean distance to obstacle boundary
                dist_xy = norm([obj.Position(1), obj.Position(2)] - [tx, ty]) - tr;
                dist_xy = max(dist_xy, 0.1); % Prevent dividing by zero
                
                % Evaluate obstacle vector constraint logic boundaries
                if dist_xy < obj.influence_dist && obj.Position(3) < (tb + th + 1.5)
                    direction = [obj.Position(1) - tx; obj.Position(2) - ty; 0];
                    if norm(direction) > 0
                        direction = direction / norm(direction);
                    end
                    % Standard APF force magnitude calculation function
                    rep_mag = obj.K_rep * (1/dist_xy - 1/obj.influence_dist) * (1/dist_xy^2);
                    F_rep = F_rep + rep_mag * direction;
                end
            end
            
            % 3. Synthesize cumulative state vector targets
            F_total = F_att + F_rep;
            max_force = 4.0;
            if norm(F_total) > max_force
                F_total = (F_total / norm(F_total)) * max_force;
            end
            
            % Translate processed steering forces into direct local path increments
            computedGoal = obj.Position + F_total * 1.2;
            
            % 4. Standard kinematic error execution loop updates
            error = computedGoal - obj.Position;
            desiredVel = obj.Kp * error;
            
            velNorm = norm(desiredVel);
            if velNorm > obj.MaxSpeed
                desiredVel = (desiredVel / velNorm) * obj.MaxSpeed;
            end
            
            obj.Velocity = desiredVel;
            obj.Position = obj.Position + obj.Velocity * obj.dt;
            
            if norm(obj.Velocity(1:2)) > 0.1
                obj.Yaw = atan2(obj.Velocity(2), obj.Velocity(1));
            end
        end
        
        function render(obj)
            % Structural spatial transformation step calculations
            R = [cos(obj.Yaw), -sin(obj.Yaw), 0;
                 sin(obj.Yaw),  cos(obj.Yaw), 0;
                 0,             0,            1];
             
            p1 = R * [ obj.armLength;  obj.armLength; 0] + obj.Position; 
            p2 = R * [-obj.armLength; -obj.armLength; 0] + obj.Position; 
            p3 = R * [ obj.armLength; -obj.armLength; 0] + obj.Position; 
            p4 = R * [-obj.armLength;  obj.armLength; 0] + obj.Position; 
            
            % Push transformed matrix structural coordinate frames to graphic handles
            set(obj.h_arm1, 'XData', [p1(1), p2(1)], 'YData', [p1(2), p2(2)], 'ZData', [p1(3), p2(3)]);
            set(obj.h_arm2, 'XData', [p3(1), p4(1)], 'YData', [p3(2), p4(2)], 'ZData', [p3(3), p4(3)]);
            
            set(obj.h_prop1, 'XData', obj.cx + p1(1), 'YData', obj.cy + p1(2), 'ZData', obj.cz + p1(3));
            set(obj.h_prop2, 'XData', obj.cx + p3(1), 'YData', obj.cy + p3(2), 'ZData', obj.cz + p3(3));
            set(obj.h_prop3, 'XData', obj.cx + p4(1), 'YData', obj.cy + p4(2), 'ZData', obj.cz + p4(3));
            set(obj.h_prop4, 'XData', obj.cx + p2(1), 'YData', obj.cy + p2(2), 'ZData', obj.cz + p2(3));
            
            obj.uav_history = [obj.uav_history, obj.Position];
            set(obj.h_path, 'XData', obj.uav_history(1,:), 'YData', obj.uav_history(2,:), 'ZData', obj.uav_history(3,:));
        end
    end
end