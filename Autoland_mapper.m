classdef Autoland_mapper < handle
    properties
        GlobalMap = [];       % Global point cloud matrix, [X, Y, Z]
        GridSize = 0.4;       % Voxel grid size (unit: meter). Smaller value means finer map, larger value means smoother visualization
        h_globalMapPlot       % Global map plot handle
    end

    methods
        function obj = Autoland_mapper(gridSize)
            % Default gridSize is 0.4
            obj.GridSize = gridSize;

            % Initialize the global map plot handle
            hold on;
            obj.h_globalMapPlot = plot3(NaN, NaN, NaN, '.', 'MarkerSize', 3);
        end

        % 核心方法：向记忆中添加新扫描的点，并进行体素化去重
        function updateMap(obj, newPoints)
            if isempty(newPoints)
                return;
            end

            % 1. 将新点云与历史全局地图合并
            combinedPoints = [obj.GlobalMap; newPoints];

            % 基于三维体素（Voxel）的快速降采样与去重
            % Map continuous coordinates to discrete grid indices
            uGrid = round(combinedPoints / obj.GridSize);
            % Find unique grid indices, remove duplicates in same grid
            [~, uniqueIdx, ~] = unique(uGrid, 'rows', 'stable');

            % Update the global map with the new points
            obj.GlobalMap = combinedPoints(uniqueIdx, :);
        end

        % Render the global map in 3D space
        function renderMap(obj)
            if ~isempty(obj.GlobalMap)
                set(obj.h_globalMapPlot, ...
                    'XData', obj.GlobalMap(:, 1), ...
                    'YData', obj.GlobalMap(:, 2), ...
                    'ZData', obj.GlobalMap(:, 3));
            end
        end
    end
end