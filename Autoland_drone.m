classdef Autoland_drone < handle
    properties
        Position = [5; 5; 10];
        Velocity = [0; 0; 0];
        Yaw = 0;
        MaxSpeed = 4.0;
        dt = 0.05;
        Kp = 0.8;
        Kd = 1.2;
        K_rep = 80.0;
        influence_dist = 6.0;
        
        State = 'ENTRY';
        TargetPos = [15; 15; 10];
        TargetLandingPos = [];
        searchTimer = 0;
        hasTarget = false;
        UsedSites = [];
        
        h_arm1, h_arm2, h_prop1, h_prop2, h_prop3, h_prop4
        h_path, uav_history, cx, cy, cz
        armLength = 1.2; propRadius = 0.4;
        h_cloud_dots
        h_target_marker
        h_body, h_radar
        h_leg1, h_leg2, h_leg3, h_leg4
        bodyVerts, bodyFaces, radarCx, radarCy, radarCz
        legOffset = 0.5;
    end
    
    methods
        function obj = Autoland_drone(startPos, initialYaw)
            obj.Position = startPos(:);
            if nargin >= 2, obj.Yaw = initialYaw; else, obj.Yaw = 0; end
            obj.TargetPos = [15; 15; 10];
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
            
            obj.h_cloud_dots = scatter3(NaN, NaN, NaN, 4, 'r', 'filled', 'MarkerEdgeAlpha', 0.5, 'MarkerFaceAlpha', 0.5);
            obj.h_path = plot3(obj.Position(1), obj.Position(2), obj.Position(3), 'b-', 'LineWidth', 2.5);
            obj.h_target_marker = scatter3(NaN, NaN, NaN, 200, 'g', 'p', 'filled', ...
                'MarkerEdgeColor', 'k', 'LineWidth', 2);
            
            bodyW = 0.6; bodyL = 0.4; bodyH = 0.25;
            obj.bodyVerts = [...
                -bodyW/2, -bodyL/2, 0;
                 bodyW/2, -bodyL/2, 0;
                 bodyW/2,  bodyL/2, 0;
                -bodyW/2,  bodyL/2, 0;
                -bodyW/2, -bodyL/2, bodyH;
                 bodyW/2, -bodyL/2, bodyH;
                 bodyW/2,  bodyL/2, bodyH;
                -bodyW/2,  bodyL/2, bodyH];
            obj.bodyFaces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
            obj.h_body = patch('Faces', obj.bodyFaces, 'Vertices', NaN(8,3), ...
                'FaceColor', [0.25 0.25 0.25], 'EdgeColor', [0.1 0.1 0.1], 'LineWidth', 0.5);
            
            [obj.radarCx, obj.radarCy, obj.radarCz] = cylinder(0.08, 8);
            obj.radarCz = obj.radarCz * 0.12 + bodyH;
            obj.h_radar = surf(NaN(2,9), NaN(2,9), NaN(2,9), 'FaceColor', [0.2 0.4 0.8], ...
                'EdgeColor', 'none', 'FaceAlpha', 0.9);
            
            obj.h_leg1 = plot3([0,0], [0,0], [0,0], 'Color', [0.5 0.5 0.5], 'LineWidth', 2.5);
            obj.h_leg2 = plot3([0,0], [0,0], [0,0], 'Color', [0.5 0.5 0.5], 'LineWidth', 2.5);
            obj.h_leg3 = plot3([0,0], [0,0], [0,0], 'Color', [0.5 0.5 0.5], 'LineWidth', 2.5);
            obj.h_leg4 = plot3([0,0], [0,0], [0,0], 'Color', [0.5 0.5 0.5], 'LineWidth', 2.5);
        end
        
        function update(obj, scanPoints, terrainF, landingSites)
            targetVel = [0; 0; 0];
            switch obj.State
                case 'ENTRY', targetVel = obj.entryBehavior();
                case 'SEARCH', targetVel = obj.searchBehavior(landingSites);
                case 'APPROACH', targetVel = obj.approachBehavior(landingSites);
                case 'DESCEND', targetVel = obj.descendBehavior(terrainF);
                case 'LANDED', obj.Velocity = [0; 0; 0]; return;
            end
            
            F_rep = obj.computeRepulsion(scanPoints);
            Kd_eff = obj.Kd;
            if strcmp(obj.State, 'DESCEND')
                F_rep = [0; 0; 0];
                Kd_eff = 4.0;
            elseif strcmp(obj.State, 'SEARCH')
                F_rep = obj.computeRepulsion(scanPoints, 0);  % 搜索阶段完全去掉切向分量，避免打转
            elseif strcmp(obj.State, 'APPROACH')
                F_rep = obj.computeRepulsion(scanPoints, 0.3);  % 接近阶段减弱切向分量
            end
            acc = targetVel - Kd_eff * obj.Velocity + F_rep;
            if norm(acc) > 6.0, acc = (acc / norm(acc)) * 6.0; end
            
            obj.Velocity = obj.Velocity + acc * obj.dt;
            speed = norm(obj.Velocity);
            if speed > obj.MaxSpeed, obj.Velocity = (obj.Velocity / speed) * obj.MaxSpeed; end
            
            obj.Position = obj.Position + obj.Velocity * obj.dt;
            
            groundH = terrainF(obj.Position(1), obj.Position(2));
            if ~isnan(groundH)
                safeAlt = 3;
                if strcmp(obj.State, 'DESCEND'), safeAlt = 0.5; end
                minZ = groundH + safeAlt;
                if obj.Position(3) < minZ
                    obj.Position(3) = minZ;
                    if obj.Velocity(3) < 0, obj.Velocity(3) = 0; end
                end
            end
            
            if norm(obj.Velocity(1:2)) > 0.1
                target_yaw = atan2(obj.Velocity(2), obj.Velocity(1));
                diff_yaw = m_unwrap(target_yaw - obj.Yaw);
                obj.Yaw = obj.Yaw + 0.1 * diff_yaw;
            end
        end
        
        function targetVel = entryBehavior(obj)
            distToCenter = norm(obj.Position(1:2) - obj.TargetPos(1:2));
            if distToCenter < 8
                obj.State = 'SEARCH';
                obj.searchTimer = 0;
                obj.hasTarget = false;
                disp('进入搜索模式...');
            end
            targetVel = obj.targetAttraction(obj.TargetPos);
        end
        
        function targetVel = searchBehavior(obj, landingSites)
            % Filter out already used sites
            availableSites = [];
            if ~isempty(landingSites)
                for i = 1:size(landingSites, 1)
                    isUsed = false;
                    if ~isempty(obj.UsedSites)
                        for j = 1:size(obj.UsedSites, 1)
                            if norm(landingSites(i, 1:2)' - obj.UsedSites(j, 1:2)') < 1.5
                                isUsed = true;
                                break;
                            end
                        end
                    end
                    if ~isUsed
                        availableSites = [availableSites; landingSites(i, :)];
                    end
                end
            end
            
            if ~isempty(availableSites) && size(availableSites, 1) > 0
                bestScore = -inf; bestIdx = 1;
                for i = 1:size(availableSites, 1)
                    score = availableSites(i, 6);  % 选面积最大的降落点
                    if score > bestScore
                        bestScore = score; bestIdx = i;
                    end
                end
                obj.TargetLandingPos = availableSites(bestIdx, 1:3)';
                obj.hasTarget = true;
                obj.State = 'APPROACH';
                disp(['发现可降落区域，位置: (' num2str(obj.TargetLandingPos(1)) ', ' ...
                    num2str(obj.TargetLandingPos(2)) ', ' num2str(obj.TargetLandingPos(3)) ')']);
                targetVel = [0; 0; 0];
                return;
            end
            
            obj.searchTimer = obj.searchTimer + obj.dt;
            t = obj.searchTimer;
            % Spiral: radius grows to 20m to cover entire map, altitude 6~10m
            searchRadius = min(20, 2.0 + t * 0.4);
            targetX = 15 + searchRadius * cos(t * 0.15);
            targetY = 15 + searchRadius * sin(t * 0.15);
            targetZ = max(6, 10 - t * 0.03);
            searchPos = [targetX; targetY; targetZ];
            targetVel = obj.targetAttraction(searchPos);
        end
        
        function targetVel = approachBehavior(obj, landingSites)
            if ~obj.hasTarget || isempty(obj.TargetLandingPos)
                obj.State = 'SEARCH'; obj.hasTarget = false; targetVel = [0;0;0]; return;
            end
            
            % Check if target has already been used
            if ~isempty(obj.UsedSites)
                for j = 1:size(obj.UsedSites, 1)
                    if norm(obj.TargetLandingPos(1:2) - obj.UsedSites(j, 1:2)') < 1.5
                        obj.State = 'SEARCH'; obj.hasTarget = false; obj.TargetLandingPos = [];
                        disp('目标已被使用，重新搜索...'); targetVel = [0;0;0]; return;
                    end
                end
            end
            
            targetValid = false;
            if ~isempty(landingSites)
                for i = 1:size(landingSites, 1)
                    if norm(landingSites(i, 1:2)' - obj.TargetLandingPos(1:2)) < 2.5
                        targetValid = true; break;
                    end
                end
            end
            if ~targetValid
                obj.State = 'SEARCH'; obj.hasTarget = false; obj.TargetLandingPos = [];
                disp('目标丢失，重新搜索...'); targetVel = [0;0;0]; return;
            end
            % APPROACH: 先水平飞到目标正上方，保持安全高度
            horizontalError = obj.TargetLandingPos(1:2) - obj.Position(1:2);
            distXY = norm(horizontalError);
            
            if distXY < 1.0
                obj.State = 'DESCEND'; disp('开始降落...'); targetVel = [0;0;0]; return;
            end
            
            % 水平移动，保持当前高度（或至少保持6m以上）
            targetVel_XY = horizontalError * 3.0;
            speedXY = norm(targetVel_XY);
            if speedXY > 3.0
                targetVel_XY = targetVel_XY / speedXY * 3.0;
            end
            
            safeZ = max(obj.TargetLandingPos(3) + 5, obj.Position(3));
            zError = safeZ - obj.Position(3);
            targetVel = [targetVel_XY; zError * 0.3];
        end
        
        function targetVel = descendBehavior(obj, terrainF)
            if ~obj.hasTarget || isempty(obj.TargetLandingPos)
                obj.State = 'SEARCH'; obj.hasTarget = false; targetVel = [0;0;0]; return;
            end
            
            % Check if target has already been used
            if ~isempty(obj.UsedSites)
                for j = 1:size(obj.UsedSites, 1)
                    if norm(obj.TargetLandingPos(1:2) - obj.UsedSites(j, 1:2)') < 1.5
                        obj.State = 'SEARCH'; obj.hasTarget = false; obj.TargetLandingPos = [];
                        disp('目标已被使用，重新搜索...'); targetVel = [0;0;0]; return;
                    end
                end
            end
            
            landingXY = obj.TargetLandingPos(1:2);
            targetZ = obj.TargetLandingPos(3) + 0.5;
            errorXY = landingXY - obj.Position(1:2);
            distXY = norm(errorXY);
            
            if distXY > 0.5
                % 水平还没有完全对准，先水平调整
                targetVel_XY = errorXY * 4.0;
                speedXY = norm(targetVel_XY);
                if speedXY > 2.0
                    targetVel_XY = targetVel_XY / speedXY * 2.0;
                end
                targetVel = [targetVel_XY; -1.0];
            else
                % 水平已对准，垂直下降
                zError = targetZ - obj.Position(3);
                vz = max(-2.0, min(1.0, zError * 1.5));
                targetVel = [errorXY * 5.0; vz];  % 强水平保持
                if abs(zError) < 1.0 && distXY <= 0.5 && norm(obj.Velocity) < 1.0
                    obj.Velocity = [0;0;0]; obj.State = 'LANDED'; obj.Position(3) = targetZ;
                    obj.UsedSites = [obj.UsedSites; obj.TargetLandingPos'];
                    disp('无人机已成功降落！'); targetVel = [0;0;0];
                end
            end
        end
        
        function F_att = targetAttraction(obj, targetPos)
            error_pos = targetPos(:) - obj.Position;
            dist_error = norm(error_pos);
            if dist_error > 5, F_att = obj.Kp * (error_pos / dist_error) * 5;
            else, F_att = obj.Kp * error_pos; end
        end
        
        function F_rep = computeRepulsion(obj, scanPoints, tangentFactor)
            if nargin < 3, tangentFactor = 0.8; end
            F_rep = [0;0;0]; if isempty(scanPoints), return; end
            numPoints = size(scanPoints, 1); rep_acc = [0;0;0]; validPoints = 0;
            for i = 1:numPoints
                dist_vector = obj.Position - scanPoints(i, :)';
                dist = norm(dist_vector); dist = max(dist, 1.5);
                if dist < obj.influence_dist && scanPoints(i, 3) > obj.Position(3) - 5
                    dir = dist_vector / dist; dir(3) = abs(dir(3)) + 0.6;
                    tangent_dir = [-dir(2); dir(1); 0];
                    combined_dir = dir + tangentFactor * tangent_dir;
                    combined_dir = combined_dir / norm(combined_dir);
                    mag = obj.K_rep * (1/dist - 1/obj.influence_dist) * (1/(dist^2));
                    rep_acc = rep_acc + mag * combined_dir; validPoints = validPoints + 1;
                end
            end
            if validPoints > 0, F_rep = rep_acc / sqrt(validPoints); end
            if obj.hasTarget && ~isempty(obj.TargetLandingPos)
                attenuation = min(1.0, norm(obj.TargetLandingPos - obj.Position) / 3.0);
                F_rep = F_rep * attenuation;
            end
        end
        
        function render(obj, scanPoints)
            R = [cos(obj.Yaw), -sin(obj.Yaw), 0; sin(obj.Yaw), cos(obj.Yaw), 0; 0, 0, 1];
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
            
            if ~isempty(obj.TargetLandingPos)
                set(obj.h_target_marker, 'XData', obj.TargetLandingPos(1), ...
                    'YData', obj.TargetLandingPos(2), 'ZData', obj.TargetLandingPos(3) + 0.5);
            else
                set(obj.h_target_marker, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
            end
            
            bodyVerts_rot = (R * obj.bodyVerts')';
            bodyVerts_world = bodyVerts_rot + obj.Position';
            set(obj.h_body, 'Vertices', bodyVerts_world);
            
            radarX = obj.radarCx * R(1,1) + obj.radarCy * R(1,2);
            radarY = obj.radarCx * R(2,1) + obj.radarCy * R(2,2);
            set(obj.h_radar, 'XData', radarX + obj.Position(1), ...
                'YData', radarY + obj.Position(2), 'ZData', obj.radarCz + obj.Position(3));
            
            leg1_s = R * [ obj.armLength;  obj.armLength; 0] + obj.Position;
            leg1_e = R * [ obj.armLength;  obj.armLength; -obj.legOffset] + obj.Position;
            leg2_s = R * [-obj.armLength; -obj.armLength; 0] + obj.Position;
            leg2_e = R * [-obj.armLength; -obj.armLength; -obj.legOffset] + obj.Position;
            leg3_s = R * [ obj.armLength; -obj.armLength; 0] + obj.Position;
            leg3_e = R * [ obj.armLength; -obj.armLength; -obj.legOffset] + obj.Position;
            leg4_s = R * [-obj.armLength;  obj.armLength; 0] + obj.Position;
            leg4_e = R * [-obj.armLength;  obj.armLength; -obj.legOffset] + obj.Position;
            
            set(obj.h_leg1, 'XData', [leg1_s(1), leg1_e(1)], 'YData', [leg1_s(2), leg1_e(2)], 'ZData', [leg1_s(3), leg1_e(3)]);
            set(obj.h_leg2, 'XData', [leg2_s(1), leg2_e(1)], 'YData', [leg2_s(2), leg2_e(2)], 'ZData', [leg2_s(3), leg2_e(3)]);
            set(obj.h_leg3, 'XData', [leg3_s(1), leg3_e(1)], 'YData', [leg3_s(2), leg3_e(2)], 'ZData', [leg3_s(3), leg3_e(3)]);
            set(obj.h_leg4, 'XData', [leg4_s(1), leg4_e(1)], 'YData', [leg4_s(2), leg4_e(2)], 'ZData', [leg4_s(3), leg4_e(3)]);
            
            obj.uav_history = [obj.uav_history, obj.Position];
            set(obj.h_path, 'XData', obj.uav_history(1,:), 'YData', obj.uav_history(2,:), 'ZData', obj.uav_history(3,:));
        end
    end
end

function dY = m_unwrap(dY)
    while dY > pi, dY = dY - 2*pi; end
    while dY < -pi, dY = dY + 2*pi; end
end
