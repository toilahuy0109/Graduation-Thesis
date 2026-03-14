%% MÔ PHỎNG 5 DRONE - KẾT HỢP ĐIỀU KHIỂN ĐỘI HÌNH VÀ GIỮ LIÊN KẾT
clear; clc; close all;

%% ========================================================================
%  1. THAM SỐ
%% ========================================================================
n_drones = 5;
dim = 3;
dt = 0.02;
T = 60;
t = 0:dt:T;
n_steps = length(t);

% Tham số điều khiển
alpha_form = 1.0;        % hệ số lực đội hình
beta = 50.0;             % hệ số damping
alpha_conn = 0.5;        % hệ số lực giữ liên kết
delta = 12;              % ngưỡng giao tiếp (khoảng cách tối đa)
omega_sigmoid = 1;       % độ dốc hàm sigmoid
gamma_leader = 50.0;      % hệ số bám quỹ đạo leader
beta_leader = 100.0;       % damping leader

% Quỹ đạo leader
R = 15;
h = 8;
vz = 0.5;
omega = 0.2;

%% ========================================================================
%  2. ĐỘI HÌNH MONG MUỐN
%% ========================================================================
L = 6;
H = 8;

p_rel_star = zeros(dim, n_drones);
p_rel_star(:,1) = [0; 0; 0];
p_rel_star(:,2) = [L; L; -H];
p_rel_star(:,3) = [L; -L; -H];
p_rel_star(:,4) = [-L; L; -H];
p_rel_star(:,5) = [-L; -L; -H];

%% ========================================================================
%  3. ĐỒ THỊ CỨNG 9 CẠNH
%% ========================================================================
edges = [1 2; 1 3; 1 4; 1 5; 2 3; 2 4; 3 4; 3 5; 4 5];
m = size(edges, 1);
fprintf('Số cạnh: %d (yêu cầu tối thiểu: %d)\n', m, 3*n_drones-6);

% Danh sách neighbors
neighbors = cell(n_drones, 1);
for i = 1:n_drones
    neighbors{i} = [];
    for e = 1:m
        if edges(e,1) == i
            neighbors{i} = [neighbors{i}, edges(e,2)];
        elseif edges(e,2) == i
            neighbors{i} = [neighbors{i}, edges(e,1)];
        end
    end
    neighbors{i} = sort(unique(neighbors{i}));
end

% Khoảng cách mong muốn
d_star = zeros(m, 1);
for e = 1:m
    i = edges(e,1); j = edges(e,2);
    d_star(e) = norm(p_rel_star(:,i) - p_rel_star(:,j));
end

%% ========================================================================
%  4. TẠO MA TRẬN Q (CHO GIỮ LIÊN KẾT)
%% ========================================================================
Q = create_Q(n_drones);

%% ========================================================================
%  5. KHỞI TẠO
%% ========================================================================
P = zeros(dim, n_drones, n_steps);
V = zeros(dim, n_drones, n_steps);

P(:,1,1) = [0; 0; h];
for i = 2:n_drones
    P(:,i,1) = P(:,1,1) + p_rel_star(:,i) + 3*randn(3,1);
end
V(:,:,1) = zeros(dim, n_drones);

%% ========================================================================
%  6. VÒNG LẶP MÔ PHỎNG
%% ========================================================================
fprintf('\n=== BẮT ĐẦU MÔ PHỎNG ===\n');

formation_error = zeros(n_steps, 1);
leader_error = zeros(n_steps, 1);
lambda2_history = zeros(n_steps, 1);

for k = 1:n_steps-1
    p_curr = P(:,:,k);
    v_curr = V(:,:,k);
    
    % =====================================================================
    %  LEADER (Drone 1)
    % =====================================================================
    i = 1;
    p_ref = [R*cos(omega*t(k)); R*sin(omega*t(k)); vz*t(k)];
    v_ref = [-R*omega*sin(omega*t(k)); R*omega*cos(omega*t(k)); 0];
    u_leader = -gamma_leader * (p_curr(:,i) - p_ref) - beta_leader * (v_curr(:,i) - v_ref);
    u(:,i) = u_leader;
    leader_error(k+1) = norm(p_curr(:,i) - p_ref);
    
    % =====================================================================
    %  LỰC GIỮ LIÊN KẾT (cho tất cả drone)
    % =====================================================================
    [u_conn, lambda2] = connectivity_force(p_curr, delta, omega_sigmoid, Q, alpha_conn);
    lambda2_history(k+1) = lambda2;
    
    % =====================================================================
    %  FOLLOWERS (Drone 2-5) - KẾT HỢP ĐỘI HÌNH + LIÊN KẾT
    % =====================================================================
    for i = 2:n_drones
        % Lực giữ đội hình (distance-based)
        u_form = zeros(dim, 1);
        for j = neighbors{i}
            p_ij = p_curr(:,j) - p_curr(:,i);
            dij = norm(p_ij);
            
            if dij > 1e-6
                e_idx = find_edge_index(edges, i, j);
                if e_idx > 0
                    u_form = u_form + alpha_form * (dij^2 - d_star(e_idx)^2) * p_ij;
                end
            end
        end
        
        % Damping
        u_damp = -beta * v_curr(:,i);
        
        % Tổng hợp: formation + connectivity + damping
        u(:,i) = u_form + u_conn(:,i) + u_damp;
    end
    
    % Cập nhật
    V(:,:,k+1) = V(:,:,k) + u * dt;
    P(:,:,k+1) = P(:,:,k) + V(:,:,k) * dt;
    
    % Tính sai số đội hình
    error_sum = 0;
    for e = 1:m
        i = edges(e,1); j = edges(e,2);
        dij = norm(P(:,i,k+1) - P(:,j,k+1));
        error_sum = error_sum + (dij - d_star(e))^2;
    end
    formation_error(k+1) = sqrt(error_sum / m);
    
    if mod(k, round(n_steps/10)) == 0
        fprintf('  t = %.1f s, leader err=%.3f, form err=%.3f, λ2=%.3f\n', ...
            k*dt, leader_error(k), formation_error(k), lambda2);
    end
end

fprintf('Mô phỏng hoàn tất!\n');

%% ========================================================================
%  7. VẼ KẾT QUẢ
%% ========================================================================
figure('Name', 'Formation + Connectivity', 'Position', [50, 50, 1400, 900]);

%% 7.1. Quỹ đạo 3D
subplot(2,3,[1,4]);
colors = lines(n_drones);
hold on; grid on; box on;

% Quỹ đạo tham chiếu leader
theta = 0:0.1:2*pi;
plot3(R*cos(theta), R*sin(theta), h*ones(size(theta)), 'k--', 'LineWidth', 1.5);

% Quỹ đạo thực tế
for i = 1:n_drones
    if i == 1
        plot3(squeeze(P(1,i,:)), squeeze(P(2,i,:)), squeeze(P(3,i,:)), ...
            'r-', 'LineWidth', 3, 'DisplayName', 'Leader');
    else
        plot3(squeeze(P(1,i,:)), squeeze(P(2,i,:)), squeeze(P(3,i,:)), ...
            'Color', colors(i,:), 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Follower %d', i));
    end
end

% Điểm đầu/cuối
for i = 1:n_drones
    plot3(P(1,i,1), P(2,i,1), P(3,i,1), 'o', 'Color', colors(i,:), ...
        'MarkerSize', 8, 'MarkerFaceColor', 'w');
    plot3(P(1,i,end), P(2,i,end), P(3,i,end), 's', 'Color', colors(i,:), ...
        'MarkerSize', 10, 'MarkerFaceColor', colors(i,:));
end

xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('Kết hợp formation + connectivity');
legend('Location', 'best');
view(45,30); axis equal;

%% 7.2. Sai số leader
subplot(2,3,2);
plot(t, leader_error, 'r-', 'LineWidth', 2);
xlabel('Thời gian (s)'); ylabel('Sai số (m)');
title('Sai số leader');
grid on;

%% 7.3. Sai số đội hình
subplot(2,3,3);
plot(t, formation_error, 'b-', 'LineWidth', 2);
xlabel('Thời gian (s)'); ylabel('Sai số RMS (m)');
title('Sai số đội hình');
grid on;

%% 7.4. Chỉ số liên thông λ₂
subplot(2,3,5);
plot(t, lambda2_history, 'g-', 'LineWidth', 2);
xlabel('Thời gian (s)'); ylabel('λ₂');
title('Chỉ số liên thông');
yline(0, 'r--', 'Nguy hiểm');
grid on;

%% 7.5. Khoảng cách
subplot(2,3,6);
hold on; grid on; box on;
for e = 1:m
    i = edges(e,1); j = edges(e,2);
    dist = squeeze(sqrt(sum((P(:,i,:) - P(:,j,:)).^2, 1)));
    plot(t, dist(:), 'LineWidth', 1, 'DisplayName', sprintf('d_{%d%d}', i, j));
end
yline(delta, 'r--', 'Ngưỡng', 'LineWidth', 2);
xlabel('Thời gian (s)'); ylabel('Khoảng cách (m)');
title('Khoảng cách');
legend('Location', 'best', 'NumColumns', 2);

sgtitle('Điều khiển đội hình + giữ liên kết', 'FontSize', 14, 'FontWeight', 'bold');

%% ========================================================================
%  HÀM PHỤ TRỢ
%% ========================================================================

function Q = create_Q(n)
    Q = randn(n, n-1);
    for i = 1:n-1
        Q(:,i) = Q(:,i) - (1/n) * (ones(1,n) * Q(:,i)) * ones(n,1);
        for j = 1:i-1
            Q(:,i) = Q(:,i) - (Q(:,j)' * Q(:,i)) * Q(:,j);
        end
        Q(:,i) = Q(:,i) / norm(Q(:,i));
    end
end

function e_idx = find_edge_index(edges, i, j)
    e_idx = 0;
    for e = 1:size(edges,1)
        if (edges(e,1) == i && edges(e,2) == j) || ...
           (edges(e,1) == j && edges(e,2) == i)
            e_idx = e;
            return;
        end
    end
end

function A = adjacency_sigmoid(P, delta, omega)
    [~, n] = size(P);
    A = zeros(n);
    for i = 1:n
        for j = i+1:n
            d_ij = norm(P(:,i) - P(:,j));
            a_ij = 1 / (1 + exp(-omega * (delta - d_ij)));
            A(i,j) = a_ij;
            A(j,i) = a_ij;
        end
    end
end

function L = laplacian_from_adjacency(A)
    D = diag(sum(A, 2));
    L = D - A;
end

function M = compute_M(P, delta, omega, Q)
    A = adjacency_sigmoid(P, delta, omega);
    L = laplacian_from_adjacency(A);
    M = Q' * L * Q;
end

function [u_conn, lambda2] = connectivity_force(P, delta, omega, Q, alpha_conn)
    [dim, n] = size(P);
    u_conn = zeros(dim, n);
    
    % Tính M
    M = compute_M(P, delta, omega, Q);
    detM = det(M);
    if detM < 1e-6
        detM = 1e-6;
    end
    
    % Tính λ₂
    A = adjacency_sigmoid(P, delta, omega);
    L = laplacian_from_adjacency(A);
    e = eig(L);
    e = sort(e);
    lambda2 = e(2);
    
    % Tính lực bằng sai phân số (đơn giản hóa)
    eps = 1e-4;
    for i = 1:n
        for d = 1:dim
            P_plus = P;
            P_plus(d,i) = P_plus(d,i) + eps;
            M_plus = compute_M(P_plus, delta, omega, Q);
            
            P_minus = P;
            P_minus(d,i) = P_minus(d,i) - eps;
            M_minus = compute_M(P_minus, delta, omega, Q);
            
            dM_dp = (M_plus - M_minus) / (2*eps);
            
            u_conn(d,i) = alpha_conn * trace(M \ dM_dp) / detM^alpha_conn;
        end
    end
    
    % Giới hạn lực
    max_u_conn = 2;
    u_conn = max(min(u_conn, max_u_conn), -max_u_conn);
end