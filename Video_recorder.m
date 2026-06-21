classdef Video_recorder < handle
    properties
        vidObj          % Video object
        current_az      % Current horizontal azimuth
        az_step = 0.1;  % Angle step for each frame rotation
    end

    methods
        % Constructor: Initialize video recorder
        % videoName: Name of the video file to save
        % start_az: Initial horizontal azimuth (default -45)
        function obj = Video_recorder(videoName, start_az)

            if nargin < 2
                start_az = -45; % Default initial horizontal azimuth
            end
            obj.current_az = start_az;
            
            % Set up video writer
            obj.vidObj = VideoWriter(videoName, 'MPEG-4');
            obj.vidObj.FrameRate = 20;
            obj.vidObj.Quality = 100;
            open(obj.vidObj);
            
            disp(['Camera ready to record: ' videoName]);
        end

        % Core method: Rotate and capture a frame
        function captureFrame(obj)
            % Update horizontal azimuth
            obj.current_az = obj.current_az + obj.az_step;
            view(obj.current_az, 35);
            drawnow; 
            
            % Capture frame and write to video
            frame = getframe(gcf);
            writeVideo(obj.vidObj, frame);
        end

        % Final method: Close video writer safely
        function closeVideo(obj)
            if ~isempty(obj.vidObj)
                close(obj.vidObj);
                disp('Video saved safely.');
            end
        end
    end
end