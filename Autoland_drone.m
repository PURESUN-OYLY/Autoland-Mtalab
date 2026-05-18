%% Drone Dynamics Model
% This model is based on the simplified 6DOF model of a drone.
% It assumes a constant mass, no drag, and no wind effects.
classdef Autoland_drone < handle
    properties
        Position = [5; 5; 5]; % Initial position [x; y; z]
        Velocity = [0; 0; 0]; % Initial velocity [vx; vy; vz]
        Yaw = 0;              % Yaw angle (rad)
        MaxSpeed = 5;         % Maximum speed (m/s) [cite: 16]
        dt = 0.05;            % Simulation step (responding to radar 20-30fps) 
        
        % PID control gains (simplified version)
        Kp = 1.5; 
    end
    
    methods
        function obj = Autoland_drone(startPos)
            obj.Position = startPos(:);
        end
        
        % Dynamic update function: move to target position
        function update(obj, targetPos)
            % 1. Calculate position error
            error = targetPos(:) - obj.Position;
            
            % To generate desired velocity
            desiredVel = obj.Kp * error;
            
            % 3. Velocity limit
            velNorm = norm(desiredVel);
            if velNorm > obj.MaxSpeed
                desiredVel = (desiredVel / velNorm) * obj.MaxSpeed;
            end
            
            % 4. Update position using velocity (Euler integration)
            obj.Velocity = desiredVel;
            obj.Position = obj.Position + obj.Velocity * obj.dt;
            
            % 5. Update yaw angle (Move direction)
            if norm(obj.Velocity(1:2)) > 0.1
                obj.Yaw = atan2(obj.Velocity(2), obj.Velocity(1));
            end
        end
    end
end
