classdef Autoland_mapper < handle
    properties
        GlobalMap = [];       % 存储全局点云矩阵 [X, Y, Z]
        GridSize = 0.4;       % 体素栅格大小（单位：米）。值越小地图越精细，值越大越流畅
        h_globalMapPlot       % 全局地图的图形句柄
    end
    
    methods
        % 构造函数
        function obj = Autoland_mapper(gridSize)
            if nargin >= 1
                obj.GridSize = gridSize;
            end
            % 初始化一个空的 3D 绘图句柄，用于后续动态更新
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
            
            % 2. 核心算法：基于三维体素（Voxel）的快速降采样与去重
            % 将连续坐标映射到离散的小方格索引上
            uGrid = round(combinedPoints / obj.GridSize);
            % 利用 unique 函数去除同一个方格内的重复点
            [~, uniqueIdx, ~] = unique(uGrid, 'rows', 'stable');
            
            % 3. 更新全局记忆
            obj.GlobalMap = combinedPoints(uniqueIdx, :);
        end
        
        % 渲染全局地图
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