clc
clear
close all

%% 导入轨迹数据
load path.mat
%path = createPath(4);
%save path.mat path;

%% 相关参数设置
k = 0.1;                % 增益系数
Kp = 1;                 % 速度P控制器系数
dt = 0.1;               % 时间间隔，s
L = 2;                  % 车辆轴距，m
RefPos = path;          % 参考轨迹
targetSpeed = 5;       % 目标速度
InitialState = [0, -2, 0, 0];   % 纵向位置、横向位置、航向角、速度

%% 主程序

% 车辆初始状态定义
state = InitialState;
state_actual = state;
target_idx = 1;

while target_idx < size(RefPos, 1) - 1
    % 寻找预瞄距离范围内最近路径点,即P1点
    [latError, target_idx] = findTargetIdx(state, RefPos);
    
    % 计算控制量
    delta = stanley_control(target_idx, state, latError, RefPos, k);
    
    % 计算加速度
    a = Kp * (targetSpeed - state(4));
    
    % 更新状态
    state = updateState(a, state, delta, dt, L);
    
    % 保存每一步的实际状态量
    state_actual(end + 1, :) = state;
end

%% 绘图
figure
plot(path(:, 1), path(:, 2), 'b');
xlabel('纵向坐标/m');
ylabel('横向坐标/m');
hold on;
for i = 1 : size(state_actual, 1)
   scatter(state_actual(i, 1), state_actual(i, 2), 150, '.r');
   pause(0.05);
end
legend('规划车辆轨迹','实际车辆轨迹')

% 保存
path_stanley = state_actual(:, 1:2);
save path_stanley.mat path_stanley;

%% 寻找预瞄距离范围内最近路径点,即P1点
function [latError, target_idx] = findTargetIdx(state, RefPos)
    for i = 1 : size(RefPos, 1)
        d(i, 1) = norm(RefPos(i, :) - state(1:2));
    end
    [latError_temp, target_idx] = min(d);
    if state(2) < RefPos(target_idx, 2) % 当前位置纵坐标小于参考点纵坐标时
        latError = -latError_temp;
    else
        latError = latError_temp;
    end
end

%% 计算控制量
function delta = stanley_control(target_idx, state, latError, RefPos, k)
    sizeOfRefPos = size(RefPos, 1);
    if target_idx < sizeOfRefPos - 5
        Point = RefPos(target_idx + 5, 1:2);    % 注意，target_idx往前第5个视为P2点
    else
        Point = RefPos(end, 1:2);
    end
    % 将角度转换到[-pi, pi]之间
    theta_phi = angleTransfer(atan((Point(2) - state(2)) / (Point(1) - state(1))) - state(3));
    theta_y = atan(k * latError / state(4));
    
    % 前轮转角
    delta = theta_phi + theta_y;
end

%% 更新状态
function state_new = updateState(a, state_old, delta, dt, L)
    state_new(1) = state_old(1) + state_old(4) * cos(state_old(3)) * dt;   % 纵向坐标
    state_new(2) = state_old(2) + state_old(4) * sin(state_old(3)) * dt;   % 横向坐标
    state_new(3) = state_old(3) + state_old(4) * dt * tan(delta) / L;      % 航向角
    state_new(4) = state_old(4) + a * dt;                                  % 纵向速度
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

%% 生成参考轨迹
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