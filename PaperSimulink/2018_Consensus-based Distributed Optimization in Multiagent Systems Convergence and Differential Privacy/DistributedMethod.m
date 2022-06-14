function [state, output] = DistributedMethod(n, A, f, alpha, gradient,x)
%% Distributed Method 算法，该算法无隐私保护，用于对比实验
% n     智能体数量
% A     邻接关系矩阵
% x     状态
% y     输出
% f(i)为智能体i的函数

    %% 参数初始化
    K = 10;        % 计算次数K
    state = zeros(K, n);         % 存储状态值
    output = zeros(K, n);        % 存储输出值
    grad_value = zeros(K, n);    % 存储梯度值
    x0 = 10 * rand(1, n); % 智能体i的初始状态xi
    state(1, :) = x0;
    for i = 1 : n
        x = x0(i);
        y0 = eval(gradient(i));    % 计算在x0处的梯度值
        output(1, i) = y0;
        grad_value(1, i) = y0;
    end
    
    k = 1;
    while k < K     % 迭代循环
        
        % 更新状态和输出
        for i = 1 : n
            state(k + 1, i) = A(:, i)' * state(k, :)' - alpha * output(k, i);
            x = state(k + 1, i);
            grad_value(k + 1, i) = eval(gradient(i));
            output(k + 1, i) = A(:, i)' * output(k, :)' + grad_value(k + 1, i) - grad_value(k, i); 
        
        end
        
%         % 更新梯度值
%         for i = 1 : n
%             x = state(k + 1, i);
%             grad_value(k + 1, i) = eval(gradient(i));
%         end
%         
%         % 更新输出
%         for i = 1 : n
%             output(k + 1, i) = A(:, i)' * output(k, :)' + grad_value(k + 1, i) - grad_value(k, i); 
%         end
        k = k + 1;
    end
end

