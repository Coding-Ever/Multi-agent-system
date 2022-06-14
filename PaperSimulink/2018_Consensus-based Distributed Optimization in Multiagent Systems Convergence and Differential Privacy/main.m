%% 主函数，测试参数设置
clc
close all
clear
%% 参数
n = 5;                  % 智能体数量
A = [0, 1, 1, 0, 1;     % 智能体间的拓扑结构
     1, 0, 0, 0, 1;
     1, 0, 0, 0, 1;
     0, 0, 0, 0, 1;
     1, 1, 1, 1, 0];
 m = 10 * rand(1, n); % 产生0-10的随机数mi，用于生成fi函数
 alpha = 0.01;          %  学习率
 %d = 1;
 d_omega = 1;
 d_eta = 1;
 q = [0.995, 0.994, 0.992, 0.99, 0.986, 0.98, 0.97, 0.96, 0.96, 0.93, 0.8, 0.3];    % 衰减率
 d = [0.1, 0.5, 1, 1.4, 2, 2.6, 3,3.8, 4];  
 
 % 声明函数
 syms x;
 for i = 1 : n
     f(i) = (x - m(i))^2;
 end
 % 求梯度
 for i = 1 : n
    gradient(i) = jacobian(f(i),x);     %求梯度，计算雅可比矩阵
 end
 
 [state1, output1] = DistributedMethod(n, A, f, alpha, gradient, x);
 figure;
 t = 0 : 1 : 100;
 plot(state1(:, 1), 'b--'); hold on;
 plot(state1(:, 2), 'r:'); hold on;
 plot(state1(:, 3), 'y'); hold on;
 plot(state1(:, 4), 'g^'); hold on;
 plot(state1(:, 5), 'co');
 hold on;