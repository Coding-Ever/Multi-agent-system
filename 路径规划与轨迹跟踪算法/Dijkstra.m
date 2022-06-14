clc
clear
close all

%% Dijkstra�㷨ʵ��·���滮
% ���nodes(4)�����յ�nodes(1)����СȨ��·��

%% ͼ�Ķ���
% ���ݽڵ���ڽ��ڵ㼰Ȩֵ�������ڵ�Ԫ������
% nodes(i,:) = {i,[nodes(i)���ڽ��ڵ�],[�ڽ��ڵ��Ӧ��Ȩֵ]}
nodes = cell(0);
nodes(1,:) = {1, [2, 6, 7], [12, 16, 14]};
nodes(2,:) = {2, [1, 3, 6], [12, 7, 10]};
nodes(3,:) = {3, [2, 4, 5], [10, 3, 6]};
nodes(4,:) = {4, [3, 5], [3, 4]};
nodes(5,:) = {5, [3, 4, 6, 7], [5, 4, 2, 8]};
nodes(6,:) = {6, [1, 2, 3, 5, 7], [16, 7, 6, 2, 9]};
nodes(7,:) = {7, [1, 5, 6], [14, 8, 9]};

%% �㷨��ʼ��
% S/U�ĵ�һ�б�ʾ�ڵ���
% ����S���ڶ��б�ʾ��Դ�ڵ㵽���ڵ�����õ���С���룬�������
% ����U���ڶ��б�ʾ��Դ�ڵ㵽���ڵ���ʱ��õ���С���룬���ܻ����
S = [4, 0];
U(:, 1) = [1, 2, 3, 5, 6, 7];
U(:, 2) = [inf, inf, 3, 4, inf, inf];

% ����·������������·���ĳ�ʼ��
% ��һ�б�ʾ�ڵ���
% �ڶ��б�ʾ��Դ�ڵ㵽���ڵ��·��
path_opt = cell(7, 2);
path_opt(4,:) = {4, 4};
path_temp = cell(7, 2);
path_temp(3,:) = {3, [4, 3]};
path_temp(4,:) = {4, 4};
path_temp(5,:) = {5, [4, 5]};

%% ѭ���������нڵ�
while ~isempty(U)
    % ��U�������ҳ���ǰ��С����ֵ�����Ӧ�Ľڵ��ţ����Ƴ��ýڵ���S������
    [distMin, indexMin] = min(U(:, 2));
    nodeMin = U(indexMin, 1);
    S(end+1, :) = [nodeMin, distMin];
    U(indexMin,:) = [];
    
    % ����С����ڵ���ӵ�����·������
    path_opt(nodeMin,:) = path_temp(nodeMin, :);
    
    % ���α�����С����ڵ���ڽ��ڵ㣬�ж��Ƿ���U�����и����ڽ��ڵ�ľ���ֵ
    for i = 1 : length(nodes{nodeMin, 2})
       % ��Ҫ�жϵĽڵ�
       nodeTemp = nodes{nodeMin, 2}(i);
       % �ҵ�U�����нڵ�nodeTemp������ֵ
       indexTemp = find(nodeTemp == U(:, 1));
       % �ж��Ƿ����
       if ~isempty(indexTemp)
           if distMin + nodes{nodeMin, 3}(i) < U(indexTemp, 2)
               U(indexTemp, 2) = distMin + nodes{nodeMin, 3}(i);
               % ������ʱ����·��
               path_temp{nodeTemp, 1} = nodeTemp;
               path_temp{nodeTemp, 2} = [path_opt{nodeMin, 2}, nodeTemp];
           end
       end
    end
end
