clc
clear
close all

%% 导入轨迹数据
load path.mat

%% 参数设置
dt = 0.1;
L = 2.9;
Q = [1, 0, 0;
     0, 1, 0;
     0, 0, 1];
R = eye(2) * 2;
refSpeed = 40 / 3.6;
refDelta = 0;

%% 轨迹处理
% 定义参考轨迹
refPos_x = path(:, 1);
refPos_y = path(:, 2);
refPos = [refPos_x, refPos_y];

% 计算一阶导数
for i = 1 : length(refPos_x) - 1
    refPos_d(i) = (refPos(i + 1, 2) - refPos(i, 2)) / (refPos(i + 1, 1) - refPos(i, 1));
end
refPos_d(end + 1) = refPos_d(end);

% 计算二阶导数
for i = 2 : length(refPos_x) - 1
    refPos_dd(i) = (refPos(i + 1, 2) - 2 * refPos(i, 2) + refPos(i - 1, 2)) / (0.5*(-refPos(i - 1, 1) + refPos(i, 1))^2 + (refPos(i + 1, 1) - refPos(i, 1))^2);
end
refPos_dd(1) = refPos_dd(2);
refPos_dd(length(refPos_x)) = refPos_dd(length(refPos_x) - 1);

% 计算曲率
for i = 1 : length(refPos_x) - 1
    k(i) = (refPos_dd(i)) / (1 + refPos_d(i)^2)^(1.5);
end

refPos_x = refPos_x';
refPos_y = refPos_y';
refPos_yaw = atan(refPos_d');
refPos_k = k';

%% 主程序
pos_x = 0;
pos_y = 0;
pos_yaw = 0;
v = 10;
Delta = 0;

pos_actual = [pos_x, pos_y];
v_actual = v;
Delta_actual = 0;

% 循环
idx = 1;
while idx < length(refPos_x) - 1;
    % 寻找参考轨迹最近的点
    idx = calc_target_index(pos_x, pos_y, refPos_x, refPos_y);
    
    % LQR控制器
    [v_delta, delta, yaw_error] = LQR_Control(idx, pos_x, pos_y, pos_yaw, refPos_x, refPos_y, refPos_yaw, dt, v, L, Q, R);
    
    % 更新状态
    [pos_x, pos_y, pos_yaw, v, Delta] = updateState(pos_x, pos_y, pos_yaw, v, v_delta, delta, dt, L, refSpeed, refDelta);
    pos_actual(end + 1, :) = [pos_x, pos_y];
    v_actual(end + 1, :) = v;
    Delta_actual(end + 1) = delta;
end

%% 绘图
figure
plot(path(:, 1), path(:, 2), 'b');
xlabel('纵向坐标/m');
ylabel('横向坐标/m');
hold on;
for i = 1 : size(pos_actual, 1)
   scatter(pos_actual(i, 1), pos_actual(i, 2), 150, '.r');
   pause(0.05);
end
legend('规划车辆轨迹','实际车辆轨迹')

%% 寻找参考轨迹最近的点
function target_idx = calc_target_index(pos_x, pos_y, refPos_x, refPos_y)
    i = 1 : length(refPos_x) - 1;
    dist = sqrt((refPos_x(i) - pos_x).^2 + (refPos_y(i) - pos_y).^2);
    [~, target_idx] = min(dist);
end

%% LQR控制器
function [v_delta, delta, yaw_error] = LQR_Control(idx, pos_x, pos_y, pos_yaw, refPos_x, refPos_y, refPos_yaw, dt, v, L, Q, R)
    % 求位置、航向角状态量计算
    x_error = pos_x - refPos_x(idx);
    y_error = pos_y - refPos_y(idx);
    yaw_error = angleTransfer(pos_yaw - refPos_yaw(idx));
    X(1,1) = x_error;
    X(2,1) = y_error;
    X(3,1) = yaw_error;
    
    % 由状态方程矩阵系数，计算K
    A = [1, 0, -dt * v * sin(pos_yaw);
         0, 1, dt * v * cos(pos_yaw);
         0, 0, 1];
    B = [dt * cos(pos_yaw), 0;
         dt * sin(pos_yaw), 0;
         dt * tan(pos_yaw) / L, dt * v / (L * cos(pos_yaw)^2)];
     
     K = calcu_K(A, B, Q, R);
     
     % 获取前轮速度变化量、前轮转角变化量两个控制变量
     u = -K * X;
     v_delta = u(1);
     delta = angleTransfer(u(2));
end

%% 计算增益K
function K = calcu_K(A, B, Q, R)
    % 终止条件定义
    iter_max = 500;
    epsilon = 0.01;
    
    % 循环
    P_old = Q;
    for i = 1 : iter_max
        P_new = A'* P_old*A - A'*P_old*B / (R + B'*P_old*B) * B'*P_old*A + Q;
        if abs(P_new - P_old) < epsilon
            break;
        else
            P_old = P_new;
        end
    end
    
    P = P_new;
    K = (B' * P * B + R) \ (B' * P * A);
    
end

%% 更新状态
function [x, y, yaw, v, Delta] = updateState(x, y, yaw, v, v_delta, delta, dt, L, refSpeed, refDelta)
    Delta = refDelta + delta;
    x = x + v * cos(yaw) * dt;
    y = y + v * sin(yaw) * dt;
    yaw = yaw + v / L * tan(Delta) * dt;
    v = refSpeed + v_delta;
end

%% 角度转换到[-pi, pi]之间
function angle = angleTransfer(angle)
    if(angle > pi)
        angle = angle - 2 * pi;
    elseif (angle < -pi)
        angle = angle + 2 * pi;
    else
        angle = angle;
    end
end

