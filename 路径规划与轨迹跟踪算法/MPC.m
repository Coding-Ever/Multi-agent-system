clc
clear
close all

%% ����켣����
load path.mat

%% ��������
Kp = 1.0;
dt = 0.1;
L = 2.9;
max_steer = 60 * pi / 180;
target_v = 40 / 3.6;

%% �켣����
% ����ο��켣
refPos_x = path(:, 1);
refPos_y = path(:, 2);
refPos = [refPos_x, refPos_y];

% ����һ�׵���
for i = 1 : length(refPos_x) - 1
    refPos_d(i) = (refPos(i + 1, 2) - refPos(i, 2)) / (refPos(i + 1, 1) - refPos(i, 1));
end
refPos_d(end + 1) = refPos_d(end);

% ������׵���
for i = 2 : length(refPos_x) - 1
    refPos_dd(i) = (refPos(i + 1, 2) - 2 * refPos(i, 2) + refPos(i - 1, 2)) / (0.5*(-refPos(i - 1, 1) + refPos(i, 1))^2 + (refPos(i + 1, 1) - refPos(i, 1))^2);
end
refPos_dd(1) = refPos_dd(2);
refPos_dd(length(refPos_x)) = refPos_dd(length(refPos_x) - 1);

% ��������
for i = 1 : length(refPos_x) - 1
    k(i) = (refPos_dd(i)) / (1 + refPos_d(i)^2)^(1.5);
end

refPos_x = refPos_x';
refPos_y = refPos_y';
refPos_yaw = atan(refPos_d');
refPos_k = k';

%% ��ͼ
figure
plot(path(:, 1), path(:, 2), 'b');
xlabel('��������/m');
ylabel('��������/m');
hold on;

%% ������
x = 0.1;
y = -0.1;
yaw = 0.1;
v = 0.1;
U = [0.01; 0.01];
pos_actual = [x, y];

ind = 0;
while ind < length(refPos_x)
    % ����MPC������
    [Delta, v, ind, e, U] = MPC_Control(x, y, yaw, refPos_x, refPos_y, refPos_yaw, refPos_k, dt, L, U, target_v);
    
    % ���̫���˳�����
    if abs(e) > 3
        fprintf('�������˳�����\n');
       break; 
    end
    
    % �ٶ�P������
    a = Kp * (target_v - v);
    
    % ����״̬����
    [x, y, yaw, v] = updateState(x, y, yaw, v, a, Delta, dt, L, max_steer);
    pos_actual(end + 1, :) = [x, y];
    
    % �����ٹ켣ͼ
    plot(x, y, 'ro');
    pause(0.01);
end

%% ����
path_MPC = pos_actual;
save path_MPC.mat path_MPC;

%% ����״̬����
function [x, y, yaw, v] = updateState(x, y, yaw, v, a, Delta, dt, L, max_steer)
    Delta = max(min(max_steer, Delta), -max_steer);
    x = x + v * cos(yaw) * dt;
    y = y + v * sin(yaw) * dt;
    yaw = yaw + v / L * tan(Delta) * dt;
    v = v + a * dt;
end



