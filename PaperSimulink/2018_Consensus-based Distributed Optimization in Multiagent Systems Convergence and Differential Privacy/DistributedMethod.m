function [state, output] = DistributedMethod(n, A, f, alpha, gradient,x)
%% Distributed Method �㷨�����㷨����˽���������ڶԱ�ʵ��
% n     ����������
% A     �ڽӹ�ϵ����
% x     ״̬
% y     ���
% f(i)Ϊ������i�ĺ���

    %% ������ʼ��
    K = 10;        % �������K
    state = zeros(K, n);         % �洢״ֵ̬
    output = zeros(K, n);        % �洢���ֵ
    grad_value = zeros(K, n);    % �洢�ݶ�ֵ
    x0 = 10 * rand(1, n); % ������i�ĳ�ʼ״̬xi
    state(1, :) = x0;
    for i = 1 : n
        x = x0(i);
        y0 = eval(gradient(i));    % ������x0�����ݶ�ֵ
        output(1, i) = y0;
        grad_value(1, i) = y0;
    end
    
    k = 1;
    while k < K     % ����ѭ��
        
        % ����״̬�����
        for i = 1 : n
            state(k + 1, i) = A(:, i)' * state(k, :)' - alpha * output(k, i);
            x = state(k + 1, i);
            grad_value(k + 1, i) = eval(gradient(i));
            output(k + 1, i) = A(:, i)' * output(k, :)' + grad_value(k + 1, i) - grad_value(k, i); 
        
        end
        
%         % �����ݶ�ֵ
%         for i = 1 : n
%             x = state(k + 1, i);
%             grad_value(k + 1, i) = eval(gradient(i));
%         end
%         
%         % �������
%         for i = 1 : n
%             output(k + 1, i) = A(:, i)' * output(k, :)' + grad_value(k + 1, i) - grad_value(k, i); 
%         end
        k = k + 1;
    end
end

