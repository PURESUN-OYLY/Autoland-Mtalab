classdef Autoland_drone < handle
    properties
        Position = [5; 5; 10];
        Velocity = [0; 0; 0];
        Yaw = 0;
        MaxSpeed = 4.5;       % Slightly lowered max speed for elegant curve tracing
        dt = 0.05;

        % Advanced 2nd-Order Physics Parameters
        Kp = 0.8;             % Attractor gain
        Kd = 1.2;             % Viscous friction (Damping) to prevent overshoot and jerky motions

        % K_rep = 25.0;         % Repulsion intensity
        % influence_dist = 6.0; % Proximity awareness buffer
        K_rep = 160.0;         % Repulsion intensity
        influence_dist = 10.0; % Proximity awareness buffer

        h_arm1, h_arm2, h_prop1, h_prop2, h_prop3, h_prop4
        h_path, uav_history, cx, cy, cz
        armLength = 1.2; propRadius = 0.4;
        h_cloud_dots
    end

    methods
        function obj = Autoland_drone(startPos)
            obj.Position = startPos(:);
            obj.uav_history = obj.Position;
            obj.initGraphics();
        end

        function initGraphics(obj)
            hold on;
            obj.h_arm1 = plot3([0,0], [0,0], [0,0], 'Color', [0.2 0.2 0.2], 'LineWidth', 3);
            obj.h_arm2 = plot3([0,0], [0,0], [0,0], 'Color', [0.2 0.2 0.2], 'LineWidth', 3);

            theta_circle = linspace(0, 2*pi, 12);
            obj.cx = obj.propRadius * cos(theta_circle);
            obj.cy = obj.propRadius * sin(theta_circle);
            obj.cz = zeros(size(theta_circle));

            obj.h_prop1 = fill3(obj.cx, obj.cy, obj.cz, 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
            obj.h_prop2 = fill3(obj.cx, obj.cy, obj.cz, 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
            obj.h_prop3 = fill3(obj.cx, obj.cy, obj.cz, 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
            obj.h_prop4 = fill3(obj.cx, obj.cy, obj.cz, 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

            % Dot size 4 provides an excellent balance between precision and visibility
            obj.h_cloud_dots = scatter3(NaN, NaN, NaN, 4, 'r', 'filled', 'MarkerEdgeAlpha', 0.5, 'MarkerFaceAlpha', 0.5);
            obj.h_path = plot3(obj.Position(1), obj.Position(2), obj.Position(3), 'b-', 'LineWidth', 2.5);
        end
        function update(obj, globalTargetPos, scanPoints, terrainF)
            % function update(obj, globalTargetPos, scanPoints)
            % 1. Target Attraction Force
            error_pos = globalTargetPos(:) - obj.Position;
            F_att = obj.Kp * error_pos;

            % 2. Dynamic Repulsion Forces
            F_rep = [0; 0; 0];
            if ~isempty(scanPoints)
                numPoints = size(scanPoints, 1);
                rep_acc = [0; 0; 0];

                for i = 1:numPoints
                    dist_vector = obj.Position - scanPoints(i, :)';
                    dist = norm(dist_vector);
                    % dist = max(dist, 0.8); % Strong lower limit prevents mathematical singularities
                    dist = max(dist, 1.5);

                    if dist < obj.influence_dist && scanPoints(i, 3) > obj.Position(3) - 4
                    % if dist < obj.influence_dist
                        dir = dist_vector / dist;
                        mag = obj.K_rep * (1/dist - 1/obj.influence_dist) * (1/(dist^2));
                        rep_acc = rep_acc + mag * dir;
                    end
                end

                % Dampen collective swarm force to avoid explosion
                % F_rep = rep_acc / log10(numPoints + 5);
                F_rep = rep_acc / sqrt(numPoints);
            end

            % 3. Total Force Composition with Damping Friction
            % F = ma. We assume mass m=1. Therefore Acceleration = F_total
            % Friction opposes current velocity, creating massive physical stability
            F_total = F_att + F_rep - obj.Kd * obj.Velocity;

            % Cap maximum acceleration forces
            if norm(F_total) > 6.0
                F_total = (F_total / norm(F_total)) * 6.0;
            end

            % 4. 2nd-Order Kinematic Integration (Crucial for Smoothness)
            acc = F_total;
            obj.Velocity = obj.Velocity + acc * obj.dt; % Velocity builds up slowly

            speed = norm(obj.Velocity);
            if speed > obj.MaxSpeed
                obj.Velocity = (obj.Velocity / speed) * obj.MaxSpeed;
            end

            obj.Position = obj.Position + obj.Velocity * obj.dt;
            groundH= terrainF(obj.Position(1), obj.Position(2));

            safeAltitude=3;

            if ~isnan(groundH)
                minZ = groundH + safeAltitude;
                if obj.Position(3) < minZ
                    obj.Position(3) = minZ;
                    if obj.Velocity(3) < 0
                        obj.Velocity(3) = 0;
                    end
                end
            end

            % 5. Yaw Alignment
            if norm(obj.Velocity(1:2)) > 0.1
                target_yaw = atan2(obj.Velocity(2), obj.Velocity(1));
                diff_yaw = m_unwrap(target_yaw - obj.Yaw);
                obj.Yaw = obj.Yaw + 0.1 * diff_yaw;
            end
        end

        function render(obj, scanPoints)
            R = [cos(obj.Yaw), -sin(obj.Yaw), 0;
                sin(obj.Yaw),  cos(obj.Yaw), 0;
                0,             0,            1];

            p1 = R * [ obj.armLength;  obj.armLength; 0] + obj.Position;
            p2 = R * [-obj.armLength; -obj.armLength; 0] + obj.Position;
            p3 = R * [ obj.armLength; -obj.armLength; 0] + obj.Position;
            p4 = R * [-obj.armLength;  obj.armLength; 0] + obj.Position;

            set(obj.h_arm1, 'XData', [p1(1), p2(1)], 'YData', [p1(2), p2(2)], 'ZData', [p1(3), p2(3)]);
            set(obj.h_arm2, 'XData', [p3(1), p4(1)], 'YData', [p3(2), p4(2)], 'ZData', [p3(3), p4(3)]);

            set(obj.h_prop1, 'XData', obj.cx + p1(1), 'YData', obj.cy + p1(2), 'ZData', obj.cz + p1(3));
            set(obj.h_prop2, 'XData', obj.cx + p3(1), 'YData', obj.cy + p3(2), 'ZData', obj.cz + p3(3));
            set(obj.h_prop3, 'XData', obj.cx + p4(1), 'YData', obj.cy + p4(2), 'ZData', obj.cz + p4(3));
            set(obj.h_prop4, 'XData', obj.cx + p2(1), 'YData', obj.cy + p2(2), 'ZData', obj.cz + p2(3));

            if ~isempty(scanPoints)
                set(obj.h_cloud_dots, 'XData', scanPoints(:,1), 'YData', scanPoints(:,2), 'ZData', scanPoints(:,3));
            else
                set(obj.h_cloud_dots, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
            end

            obj.uav_history = [obj.uav_history, obj.Position];
            set(obj.h_path, 'XData', obj.uav_history(1,:), 'YData', obj.uav_history(2,:), 'ZData', obj.uav_history(3,:));
        end
    end
end

function dY = m_unwrap(dY)
while dY > pi,  dY = dY - 2*pi; end
while dY < -pi, dY = dY + 2*pi; end
end