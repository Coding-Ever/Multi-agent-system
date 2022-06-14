clc
clear
close all

%% ����켣����
load path.mat
%path = createPath(4);
%save path.mat path;

%% ��ز�������
k = 0.1;                % ����ϵ��
Kp = 1;                 % �ٶ�P������ϵ��
dt = 0.1;               % ʱ������s
L = 2;                  % ������࣬m
RefPos = path;          % �ο��켣
targetSpeed = 5;       % Ŀ���ٶ�
InitialState = [0, -2, 0, 0];   % ����λ�á�����λ�á�����ǡ��ٶ�

%% ������

% ������ʼ״̬����
state = InitialState;
state_actual = state;
target_idx = 1;

while target_idx < size(RefPos, 1) - 1
    % Ѱ��Ԥ����뷶Χ�����·����,��P1��
    [latError, target_idx] = findTargetIdx(state, RefPos);
    
    % ���������
    delta = stanley_control(target_idx, state, latError, RefPos, k);
    
    % ������ٶ�
    a = Kp * (targetSpeed - state(4));
    
    % ����״̬
    state = updateState(a, state, delta, dt, L);
    
    % ����ÿһ����ʵ��״̬��
    state_actual(end + 1, :) = state;
end

%% ��ͼ
figure
plot(path(:, 1), path(:, 2), 'b');
xlabel('��������/m');
ylabel('��������/m');
hold on;
for i = 1 : size(state_actual, 1)
   scatter(state_actual(i, 1), state_actual(i, 2), 150, '.r');
   pause(0.05);
end
legend('�滮�����켣','ʵ�ʳ����켣')

% ����
path_stanley = state_actual(:, 1:2);
save path_stanley.mat path_stanley;

%% Ѱ��Ԥ����뷶Χ�����·����,��P1��
function [latError, target_idx] = findTargetIdx(state, RefPos)
    for i = 1 : size(RefPos, 1)
        d(i, 1) = norm(RefPos(i, :) - state(1:2));
    end
    [latError_temp, target_idx] = min(d);
    if state(2) < RefPos(target_idx, 2) % ��ǰλ��������С�ڲο���������ʱ
        latError = -latError_temp;
    else
        latError = latError_temp;
    end
end

%% ���������
function delta = stanley_control(target_idx, state, latError, RefPos, k)
    sizeOfRefPos = size(RefPos, 1);
    if target_idx < sizeOfRefPos - 5
        Point = RefPos(target_idx + 5, 1:2);    % ע�⣬target_idx��ǰ��5����ΪP2��
    else
        Point = RefPos(end, 1:2);
    end
    % ���Ƕ�ת����[-pi, pi]֮��
    theta_phi = angleTransfer(atan((Point(2) - state(2)) / (Point(1) - state(1))) - state(3));
    theta_y = atan(k * latError / state(4));
    
    % ǰ��ת��
    delta = theta_phi + theta_y;
end

%% ����״̬
function state_new = updateState(a, state_old, delta, dt, L)
    state_new(1) = state_old(1) + state_old(4) * cos(state_old(3)) * dt;   % ��������
    state_new(2) = state_old(2) + state_old(4) * sin(state_old(3)) * dt;   % ��������
    state_new(3) = state_old(3) + state_old(4) * dt * tan(delta) / L;      % �����
    state_new(4) = state_old(4) + a * dt;                                  % �����ٶ�
end

%% �Ƕ�ת����[-pi, pi]֮��
function angle = angleTransfer(angle)
    if(angle > pi)
        angle = angle - 2 * pi;
    elseif (angle < -pi)
        angle = angle + 2 * pi;
    else
        angle = angle;
    end
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
           path(i, 2) = -5*cos(0.03*path(i, 1));
           i = i + 1;
        end
    end
end