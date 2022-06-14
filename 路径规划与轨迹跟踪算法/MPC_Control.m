function [Delta_real, v_real, idx, error_y, U] = MPC_Control(x, y, yaw, refPos_x, refPos_y, refPos_yaw, refPos_k, dt, L, U, target_v)
    %% MPC控制器实现
    Nx = 3;     % 状态变量的数量
    Nu = 2;     % 控制量的数量
    Np = 60;    % 预测步长
    Nc = 30;    % 控制步长
    row = 10;   % 松弛因子
    Q = 100 * eye(Np * Nx);     % (Np * Nx)*(Np * Nx)
    R = 1 * eye(Nc * Nu);       % (Nc * Nu)*(Nc * Nu)
    
    % 控制约束条件
    umin = [-0.2; -0.54];
    umax = [0.2; 0.332];
    delta_umin = [-0.05; -0.64];
    delta_umax = [0.05; 0.64];
    
    %% 原运动学误差状态空间方程的相关矩阵
    % 计算参考控制量
    [idx, error_y] = calc_target_index(x, y, refPos_x, refPos_y);
    curvature = refPos_k(idx);
    Delta_r = atan(L * curvature);
    v_r = target_v;
    
    %% 实际状态量与参考状态量
    X_real = [x, y, yaw];
    Xr = [refPos_x(idx), refPos_y(idx), refPos_yaw(idx)];
    
    %% a,b矩阵
    a = [1, 0, -dt * v_r * sin(yaw);
         0, 1, dt * v_r * cos(yaw);
         0, 0, 1];
    b = [dt * cos(yaw), 0;
         dt * sin(yaw), 0;
         dt * tan(yaw) / L, dt * v_r / (L * cos(Delta_r)^2)];
     
     %% 新的状态空间方程相关矩阵
     % 新的状态变量
     kesi = zeros(Nx + Nu, 1);      % (Nx + Nu)* 1
     kesi(1 : Nx) = X_real - Xr;
     kesi(Nx + 1 : end) = U;
     
     % 新的A矩阵
     A_cell = cell(2, 2);
     A_cell{1, 1} = a;
     A_cell{1, 2} = b;
     A_cell{2, 1} = zeros(Nu, Nx);
     A_cell{2, 2} = eye(Nu);
     A = cell2mat(A_cell);          % (Nx + Nu) * (Nx + Nu)
     
     % 新的B矩阵
     B_cell = cell(2, 1);
     B_cell{1, 1} = b;
     B_cell{2, 1} = eye(Nu);
     B = cell2mat(B_cell);          % (Nx + Nu) * Nu
     
     % 新的C矩阵
     C_cell = cell(1, 2);
     C_cell{1, 1} = eye(Nx);
     C_cell{1, 2} = zeros(Nx, Nu);
     C = cell2mat(C_cell);          % Nx * (Nx + Nu)
     
     % Phi矩阵
     Phi_cell = cell(Np, 1);
     for i = 1 : Np
         Phi_cell{i, 1} = C * A^i;
     end
     Phi = cell2mat(Phi_cell); 
     
     % Theta矩阵
     Theta_cell = cell(Np, 1);
     for i = 1 : Np
         for j = 1 : Nc
             if j <= i
                 Theta_cell{i, j} = C * A^(i-j)*B;  
             else
                 Theta_cell{i, j} = zeros(Nx, Nu);
             end
         end
     end
     Theta = cell2mat(Theta_cell);
     
     %% 二次型目标函数的相关矩阵
     % H矩阵
     H_cell = cell(2, 2);
     H_cell{1, 1} = Theta' * Q * Theta + R;     % (Nu + Nc) * (Nu + Nc)
     H_cell{1, 2} = zeros(Nu * Nc, 1);
     H_cell{2, 1} = zeros(1, Nu * Nc);
     H_cell{2, 2} = row;    % 为了使得二次型目标函数有解，这里加入松弛因子
     H = cell2mat(H_cell);                      % (Nu + Nc + 1) * (Nu + Nc + 1)
     
     % E矩阵
     E = Phi * kesi;        % (Nx + Np) * 1
     
     % g矩阵
     g_cell = cell(1, 1);
     g_cell{1, 1} = E' * Q * Theta;             % (Nu + Nc) * 1，行数为了和H的列数相匹配，添加一列0
     g_cell{1, 2} = 0;
     g = cell2mat(g_cell);                      % (Nu + Nc + 1) * 1
     
     %% 约束条件的相关矩阵
     % A_I矩阵
     At = zeros(Nc, Nc);           % 下三角方阵
     for i = 1 : Nc
         At(i, 1 : i) = 1; 
     end
     A_I = kron(At, eye(Nu));      % (Nu * Nc) * (Nu * Nc)
     
     % Ut矩阵
     Ut = kron(ones(Nc, 1), U);     % (Nu * Nc) * 1
     
     % 控制量与控制量变化量的约束
     Umin = kron(ones(Nc, 1), umin);
     Umax = kron(ones(Nc, 1), umax);
     delta_Umin = kron(ones(Nc, 1), delta_umin);
     delta_Umax = kron(ones(Nc, 1), delta_umax);
     
     % 对于二次型函数不等式的约束Ax<=b的矩阵A
     A_cons_cell = {A_I, zeros(Nu * Nc, 1);     % 列数为了和H的列数匹配，新添加一列0
         -A_I, zeros(Nu * Nc, 1)};
     A_cons = cell2mat(A_cons_cell);            % (Nu * Nc * 2) * (Nu * Nc + 1)
     
     % 用于二次型函数不等式的约束Ax<=b的矩阵b
     b_cons_cell = {Umax - Ut;
                    -Umin + Ut};
     b_cons = cell2mat(b_cons_cell);
     
     %delta_U的上下界约束
     lb = [delta_Umin; 0];
     ub = [delta_Umax; 1];
     
     %% 开始求解过程
     options = optimoptions('quadprog', 'Display', 'iter', 'MaxIterations', 100, 'TolFun', 1e-16);
     delta_U = quadprog(H, g, A_cons, b_cons, [], [], lb, ub, [], options);     % (Nu * Nc + 1) * 1
     
     %% 计算输出
     
     % 只选取求解的delta_U的第一组控制量。注意：这里是v_tilde的变化量和Delta_tilde的变化量
     delta_v_tilde = delta_U(1);
     delta_Delta_tilde = delta_U(2);
     
     % 更新这一时刻的控制量。注意，这里的控制量是v_tilde和Delta_tilde，而不是真正的v和Delta
     U(1) = kesi(4) + delta_v_tilde;
     U(2) = kesi(5) + delta_Delta_tilde;
     
     % 求取真正的控制量v_real 和Delta_real
     v_real = U(1) + v_r;
     Delta_real = U(2) + Delta_r;
end

function [target_idx, error_y] = calc_target_index(x, y, refPos_x, refPos_y)
    i = 1 : length(refPos_x) - 1;
    dist = sqrt((refPos_x(i) - x).^2 + (refPos_y(i) - y).^2);
    [value , target_idx] = min(dist);
    
    if y < refPos_y(target_idx)
        error_y = -value;
    else
        error_y = value;
    end
end

