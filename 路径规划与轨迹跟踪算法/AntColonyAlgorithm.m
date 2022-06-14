%% 蚁群算法实现路径规划
% 模拟自然界中蚂蚁的觅食行为：
% 采用信息素浓度来表征路径的远近，信息素浓度越高，表示对应的路径距离越短
% 通常，蚂蚁会优先以较大的概率选择信息素浓度较高的路径，并释放一定量的信息素，
% 以增强该条路径上的信息素浓度，形成正反馈。

%% 算法思想：
% 用蚂蚁行走的路径表示待优化问题的可行解，整个蚂蚁群体的所有路径构成得优化问题的解空间
% 路径较短的蚂蚁释放的信息素较多，随着时间的推移，较短路径上积累的信息素浓度主键增高，选择该路径的马一个数也越来越多
% 最终，整个蚂蚁会在正反馈的作用下集中到最佳的路径上，此时对应的辨识得优化问题的最优解

%% 
clc
clear 
close all

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

% 始末节点
node_start = 4;
node_end = 1;

%% 蚁群相关定义
m = 50;                             % 蚂蚁数量
n = size(nodes, 1);                 % 节点数量
alpha = 1;                          % 信息素重要程度因子
beta = 5;                           % 启发函数重要程度因子
rho = 0.1;                          % 信息素挥发因子
Q = 1;                              % 常数

%% 迭代过程相关参数初始化定义
iter = 1;                           % 迭代次数初值
iter_max = 100;                     % 最大迭代次数
Route_best = cell(iter_max, 1);     % 各代最佳路径
Length_best = zeros(iter_max, 1);   % 各代最佳路径的长度
Length_avg = zeros(iter_max, 1);    % 各代路径的平均长度

%% 将信息素、挥发因子一并放入nodes中
Delta_Tau_initial = nodes(:,1:2);
for i = 1 : size(nodes, 1)
   nodes{i,4} = ones(1,length(nodes{i,3}));     % 信息素
   nodes{i,5} = 1./nodes{i,3};                  % 挥发因子
   Delta_Tau_initial{i,3} = zeros(1,length(nodes{i,3}));
end

%% 迭代寻找最佳路径
while iter <= iter_max
   route = cell(0);
   %% 逐个蚂蚁选择路径
   for i = 1:m
      % 逐个节点路径选择
      neighbor = cell(0);
      node_step = node_start;
      path = node_step;
      dist = 0;
      % 当路径表里包含终结点时，该蚂蚁完成路径寻优，跳出循环
      while ~ismember(node_end, path)
          
          % 寻找邻近节点
          neighbor = nodes{node_step, 2};
          
          % 删除已经访问过的邻近节点
          idx = [];
          for k = 1:length(neighbor)
             if ismember(neighbor(k), path)
                 idx(end + 1) = k;
             end
          end
          neighbor(idx) = [];
          % 判断是否进入死胡同，若是，则直接返回到起点，重新寻路
          if isempty(neighbor)
             neighbor = cell(0);
             node_step = node_start;
             path = node_step;
             dist = 0;
             continue;
          end
          
          % 计算下一个节点的访问概率
          P = neighbor;
          for k = 1 : length(P)
             P(2,k) = nodes{node_step, 4}(k)^alpha * ...
                 nodes{node_step, 5}(k)^beta;
          end
          P(2,:) = P(2,:)/sum(P(2,:));
          
          % 轮盘赌法选择下一个访问节点
          Pc = cumsum(P(2,:));
          Pc = [0, Pc];
          randnum = rand;
          for k = 1 : length(Pc) - 1
              if randnum > Pc(k) && randnum < Pc(k + 1)
                 target_node = neighbor(k); 
              end
          end
          
          % 计算单步距离
          idx_temp = find(nodes{node_step, 2} == target_node);
          dist = dist + nodes{node_step, 3}(idx_temp);
          
          % 更新下一步的目标节点及路经集合
          node_step = target_node;
          path(end + 1) = node_step;
          
      end
      
      % 存放第i只蚂蚁的累计距离及对应路径
      Length(i,1) = dist;
      route{i,1} = path;
   end
   
   %% 计算这一代的m只蚂蚁中最短距离及对应的路径
   if iter == 1
       [min_Length, min_index] = min(Length);
       Length_best(iter) = min_Length;
       Length_avg(iter) = mean(Length);
       Route_best{iter,1} = route{min_index,1};
   else
       [min_Length, min_index] = min(Length);
       Length_best(iter) = min(Length_best(iter - 1),min_Length);
       Length_avg(iter) = mean(Length);
       if Length_best(iter) == min_Length
           Route_best{iter,1} = route{min_index,1};
       else
           Route_best{iter,1} = Route_best{iter - 1,1};
       end  
   end
   
   %% 更新信息素
   % 计算每一条路径上的经过的蚂蚁留下的信息素
   Delta_Tau = Delta_Tau_initial;
   % 逐个蚂蚁计算
   for i = 1 : m
      % 逐个节点计算
      for j = 1 : length(route{i,1}) - 1
         node_start_temp = route{i,1}(j);
         node_end_temp = route{i,1}(j + 1);
         idx = find(Delta_Tau{node_start_temp,2} == node_end_temp);
         Delta_Tau{node_start_temp,3}(idx) = Delta_Tau{node_start_temp,3}(idx) + Q/Length(i);
      end
   end
   
   % 考虑挥发因子，更新信息素
   for i = 1 : size(nodes, 1)
       nodes{i,4} = (1 - rho) * nodes{i,4} + Delta_Tau{i,3};
   end
   
   iter = iter + 1;
    
end

%% 绘图、结果
figure
plot(1:iter_max, Length_best, 'b', 1:iter_max, Length_avg, 'r')
legend('最短距离', '平均距离')
xlabel('迭代次数')
ylabel('距离')
title('各代最短距离与平均距离对比')

%% 最优路径
[dist_min, idx] = min(Length_best);
path_opt = Route_best(idx, 1);
