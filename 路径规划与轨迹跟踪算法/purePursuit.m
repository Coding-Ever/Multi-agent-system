clc
clear
close all

%% ����켣����
load path.mat
%path = createPath(4);
%save path.mat path;

%% ��ز�������
Kv = 0.1;               % ǰ�Ӿ���ϵ��
Kp = 0.8;               % �ٶ�P������ϵ��
Ld0 = 2;                % Ԥ����������ֵ
dt = 0.1;               % ʱ������s
L = 2.9;                % ������࣬m
RefPos = path(:,1:2);
targetSpeed = 10;

%% ������
% ������ʼ״̬����
pos = [1, 1.5];%RefPos(1,:);
v = 2;
heading = 0;

pos_actual = pos;           % ��ʵλ��
heading_actual = heading;   % ��ʵ�����
v_actual = v;               % ��ʵ�ٶ�
idx_target = 1;             % Ŀ��λ������

while idx_target < size(RefPos, 1) - 1
   % Ѱ��Ԥ����뷶Χ�����·���� 
   [lookaheadPoint, idx_target] = findLookaheadPoint(pos, v, RefPos, Kv, Ld0);
   
   % ���������
   delta = pure_pursuit_control(lookaheadPoint, idx_target, pos, heading, v, RefPos, Kv, Ld0, L);
   
   % ������ٶ�
   a = Kp * (targetSpeed - v)/dt;
   
   % ����״̬����
   [pos, heading, v] = updateState(a, pos, heading, v, delta, L, dt);
   
   % ����ÿһ����ʵ��״̬��
   pos_actual(end + 1, :) = pos;
   heading_actual(end + 1, :) = heading;
   v_actual(end + 1, :) = v;
end

%% ��ͼ
figure
plot(path(:, 1), path(:, 2), 'b');
xlabel('��������/m');
ylabel('��������/m');
hold on;
for i = 1 : size(pos_actual, 1)
   scatter(pos_actual(i, 1), pos_actual(i, 2), 150, '.r');
   pause(0.05);
end
legend('�滮�����켣','ʵ�ʳ����켣')

% ����
path_PP = pos_actual;

%% �ڲο��켣���������뵱ǰλ������ĵ�,��C��
function [lookaheadPoint, idx_target] = findLookaheadPoint(pos, v, RefPos, Kv, Ld0)
    % �ҵ����뵱ǰλ������Ĳο��켣������
    sizeOfRefPos = size(RefPos, 1);
    for i = 1 : sizeOfRefPos
        dist(i, 1) = norm(RefPos(i, :) - pos);
    end
    [~, idx] = min(dist);
    
    % �Ӹõ㿪ʼ��켣ǰ���������ҵ���Ԥ������������һ���켣��
    L_steps = 0;           % �ο��켣�ϼ������ڵ���ۼƾ���
    Ld = Kv * v + Ld0;     
    while L_steps < Ld && idx < sizeOfRefPos
        L_steps = L_steps + norm(RefPos(idx + 1,:) - RefPos(idx,:));
        idx = idx + 1;
    end
    
    idx_target = idx;
    lookaheadPoint = RefPos(idx,:);
end

%% ���������
function delta = pure_pursuit_control(lookaheadPoint, idx_target, pos, heading, v, RefPos, Kv, Ld0, L)
    sizeOfRefPos = size(RefPos, 1);
    if idx_target < sizeOfRefPos
        Point_temp = lookaheadPoint;
    else
        Point_temp = RefPos(end, 1:2);
    end
    
    alpha = atan2(Point_temp(1, 2) - pos(2), Point_temp(1,1) - pos(1)) - heading;
    Ld = Kv * v + Ld0;
    
    % ǰ��ת��
    delta = atan2(2 * L * sin(alpha), Ld);
end

%% �����˶�ѧģ�͸���״̬��
function [pos_new, heading_new, v_new] = updateState(a, pos_old, heading_old, v_old, delta, wheelbase, dt)
    pos_new(1) = pos_old(1) + v_old * cos(heading_old) *dt;
    pos_new(2) = pos_old(2) + v_old * sin(heading_old) *dt;
    heading_new = heading_old + v_old *dt * tan(delta) / wheelbase;
    v_new = v_old + a * dt;
end

%% ���ɲο��켣
function [path] = createPath(mode)
    path(:, 1) = 0 : 0.1 : 100;
    if mode == 1
        for i = 1 : size(path, 1)
           path(i, 2) = 1./(1+exp(-0.1*path(i, 1)));
           i = i + 1;
        end     
    elseif mode == 2
        for i = 1 : size(path, 1)
           path(i, 2) = 5*sin(0.02*path(i, 1));
           i = i + 1;
        end
    elseif mode == 3
        for i = 1 : size(path, 1)
           path(i, 2) = path(i, 1);
           i = i + 1;
        end
    elseif mode == 4
        for i = 1 : size(path, 1)
           path(i, 2) = -2*cos(0.03*path(i, 1));
           i = i + 1;
        end
    end
end