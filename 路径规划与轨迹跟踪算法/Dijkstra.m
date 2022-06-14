clc
clear
close all

%% Dijkstra算法实现路径规划
% 起点nodes(4)到达终点nodes(1)的最小权重路径

%% 图的定义
% 根据节点的邻近节点及权值，构建节点元包数组
% nodes(i,:) = {i,[nodes(i)的邻近节点],[邻近节点对应的权值]}
nodes = cell(0);
nodes(1,:) = {1, [2, 6, 7], [12, 16, 14]};
nodes(2,:) = {2, [1, 3, 6], [12, 7, 10]};
nodes(3,:) = {3, [2, 4, 5], [10, 3, 6]};
nodes(4,:) = {4, [3, 5], [3, 4]};
nodes(5,:) = {5, [3, 4, 6, 7], [5, 4, 2, 8]};
nodes(6,:) = {6, [1, 2, 3, 5, 7], [16, 7, 6, 2, 9]};
nodes(7,:) = {7, [1, 5, 6], [14, 8, 9]};

%% 算法初始化
% S/U的第一列表示节点编号
% 对于S，第二列表示从源节点到本节点已求得的最小距离，不会更新
% 对于U，第二列表示从源节点到本节点暂时求得的最小距离，可能会更新
S = [4, 0];
U(:, 1) = [1, 2, 3, 5, 6, 7];
U(:, 2) = [inf, inf, 3, 4, inf, inf];

% 最优路径及短暂最优路径的初始化
% 第一列表示节点编号
% 第二列表示从源节点到本节点的路径
path_opt = cell(7, 2);
path_opt(4,:) = {4, 4};
path_temp = cell(7, 2);
path_temp(3,:) = {3, [4, 3]};
path_temp(4,:) = {4, 4};
path_temp(5,:) = {5, [4, 5]};

%% 循环遍历所有节点
while ~isempty(U)
    % 在U集合中找出当前最小距离值及其对应的节点编号，并移除该节点至S集合中
    [distMin, indexMin] = min(U(:, 2));
    nodeMin = U(indexMin, 1);
    S(end+1, :) = [nodeMin, distMin];
    U(indexMin,:) = [];
    
    % 将最小距离节点添加到最优路径集合
    path_opt(nodeMin,:) = path_temp(nodeMin, :);
    
    % 依次遍历最小距离节点的邻近节点，判断是否在U集合中更新邻近节点的距离值
    for i = 1 : length(nodes{nodeMin, 2})
       % 需要判断的节点
       nodeTemp = nodes{nodeMin, 2}(i);
       % 找到U集合中节点nodeTemp的索引值
       indexTemp = find(nodeTemp == U(:, 1));
       % 判断是否更新
       if ~isempty(indexTemp)
           if distMin + nodes{nodeMin, 3}(i) < U(indexTemp, 2)
               U(indexTemp, 2) = distMin + nodes{nodeMin, 3}(i);
               % 更新暂时最优路径
               path_temp{nodeTemp, 1} = nodeTemp;
               path_temp{nodeTemp, 2} = [path_opt{nodeMin, 2}, nodeTemp];
           end
       end
    end
end
