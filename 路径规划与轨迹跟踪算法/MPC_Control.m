function [Delta_real, v_real, idx, error_y, U] = MPC_Control(x, y, yaw, refPos_x, refPos_y, refPos_yaw, refPos_k, dt, L, U, target_v)
    %% MPC������ʵ��
    Nx = 3;     % ״̬����������
    Nu = 2;     % ������������
    Np = 60;    % Ԥ�ⲽ��
    Nc = 30;    % ���Ʋ���
    row = 10;   % �ɳ�����
    Q = 100 * eye(Np * Nx);     % (Np * Nx)*(Np * Nx)
    R = 1 * eye(Nc * Nu);       % (Nc * Nu)*(Nc * Nu)
    
    % ����Լ������
    umin = [-0.2; -0.54];
    umax = [0.2; 0.332];
    delta_umin = [-0.05; -0.64];
    delta_umax = [0.05; 0.64];
    
    %% ԭ�˶�ѧ���״̬�ռ䷽�̵���ؾ���
    % ����ο�������
    [idx, error_y] = calc_target_index(x, y, refPos_x, refPos_y);
    curvature = refPos_k(idx);
    Delta_r = atan(L * curvature);
    v_r = target_v;
    
    %% ʵ��״̬����ο�״̬��
    X_real = [x, y, yaw];
    Xr = [refPos_x(idx), refPos_y(idx), refPos_yaw(idx)];
    
    %% a,b����
    a = [1, 0, -dt * v_r * sin(yaw);
         0, 1, dt * v_r * cos(yaw);
         0, 0, 1];
    b = [dt * cos(yaw), 0;
         dt * sin(yaw), 0;
         dt * tan(yaw) / L, dt * v_r / (L * cos(Delta_r)^2)];
     
     %% �µ�״̬�ռ䷽����ؾ���
     % �µ�״̬����
     kesi = zeros(Nx + Nu, 1);      % (Nx + Nu)* 1
     kesi(1 : Nx) = X_real - Xr;
     kesi(Nx + 1 : end) = U;
     
     % �µ�A����
     A_cell = cell(2, 2);
     A_cell{1, 1} = a;
     A_cell{1, 2} = b;
     A_cell{2, 1} = zeros(Nu, Nx);
     A_cell{2, 2} = eye(Nu);
     A = cell2mat(A_cell);          % (Nx + Nu) * (Nx + Nu)
     
     % �µ�B����
     B_cell = cell(2, 1);
     B_cell{1, 1} = b;
     B_cell{2, 1} = eye(Nu);
     B = cell2mat(B_cell);          % (Nx + Nu) * Nu
     
     % �µ�C����
     C_cell = cell(1, 2);
     C_cell{1, 1} = eye(Nx);
     C_cell{1, 2} = zeros(Nx, Nu);
     C = cell2mat(C_cell);          % Nx * (Nx + Nu)
     
     % Phi����
     Phi_cell = cell(Np, 1);
     for i = 1 : Np
         Phi_cell{i, 1} = C * A^i;
     end
     Phi = cell2mat(Phi_cell); 
     
     % Theta����
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
     
     %% ������Ŀ�꺯������ؾ���
     % H����
     H_cell = cell(2, 2);
     H_cell{1, 1} = Theta' * Q * Theta + R;     % (Nu + Nc) * (Nu + Nc)
     H_cell{1, 2} = zeros(Nu * Nc, 1);
     H_cell{2, 1} = zeros(1, Nu * Nc);
     H_cell{2, 2} = row;    % Ϊ��ʹ�ö�����Ŀ�꺯���н⣬��������ɳ�����
     H = cell2mat(H_cell);                      % (Nu + Nc + 1) * (Nu + Nc + 1)
     
     % E����
     E = Phi * kesi;        % (Nx + Np) * 1
     
     % g����
     g_cell = cell(1, 1);
     g_cell{1, 1} = E' * Q * Theta;             % (Nu + Nc) * 1������Ϊ�˺�H��������ƥ�䣬���һ��0
     g_cell{1, 2} = 0;
     g = cell2mat(g_cell);                      % (Nu + Nc + 1) * 1
     
     %% Լ����������ؾ���
     % A_I����
     At = zeros(Nc, Nc);           % �����Ƿ���
     for i = 1 : Nc
         At(i, 1 : i) = 1; 
     end
     A_I = kron(At, eye(Nu));      % (Nu * Nc) * (Nu * Nc)
     
     % Ut����
     Ut = kron(ones(Nc, 1), U);     % (Nu * Nc) * 1
     
     % ��������������仯����Լ��
     Umin = kron(ones(Nc, 1), umin);
     Umax = kron(ones(Nc, 1), umax);
     delta_Umin = kron(ones(Nc, 1), delta_umin);
     delta_Umax = kron(ones(Nc, 1), delta_umax);
     
     % ���ڶ����ͺ�������ʽ��Լ��Ax<=b�ľ���A
     A_cons_cell = {A_I, zeros(Nu * Nc, 1);     % ����Ϊ�˺�H������ƥ�䣬�����һ��0
         -A_I, zeros(Nu * Nc, 1)};
     A_cons = cell2mat(A_cons_cell);            % (Nu * Nc * 2) * (Nu * Nc + 1)
     
     % ���ڶ����ͺ�������ʽ��Լ��Ax<=b�ľ���b
     b_cons_cell = {Umax - Ut;
                    -Umin + Ut};
     b_cons = cell2mat(b_cons_cell);
     
     %delta_U�����½�Լ��
     lb = [delta_Umin; 0];
     ub = [delta_Umax; 1];
     
     %% ��ʼ������
     options = optimoptions('quadprog', 'Display', 'iter', 'MaxIterations', 100, 'TolFun', 1e-16);
     delta_U = quadprog(H, g, A_cons, b_cons, [], [], lb, ub, [], options);     % (Nu * Nc + 1) * 1
     
     %% �������
     
     % ֻѡȡ����delta_U�ĵ�һ���������ע�⣺������v_tilde�ı仯����Delta_tilde�ı仯��
     delta_v_tilde = delta_U(1);
     delta_Delta_tilde = delta_U(2);
     
     % ������һʱ�̵Ŀ�������ע�⣬����Ŀ�������v_tilde��Delta_tilde��������������v��Delta
     U(1) = kesi(4) + delta_v_tilde;
     U(2) = kesi(5) + delta_Delta_tilde;
     
     % ��ȡ�����Ŀ�����v_real ��Delta_real
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

