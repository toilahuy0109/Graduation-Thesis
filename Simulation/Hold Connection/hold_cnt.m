%% ========================================================================
%  MÔ PHỎNG LUẬT GIỮ LIÊN KẾT CHO DRONE
%  Dựa trên lý thuyết: Φ_c(p) = 1/[det(M)]^α với M = Q^T L(p) Q
%  Luật điều khiển: u = -∇Φ_c(p)
%  ========================================================================

clear; clc; close all;

%% ========================================================================
%  1. THAM SỐ MÔ PHỎNG
%% ========================================================================
n_drones = 4;           % số drone
dim = 3;                 % không gian 3D
delta = 10;              % ngưỡng giao tiếp (khoảng cách tối đa)
omega = 1;               % độ dốc hàm sigmoid
alpha = 2;               % tham số hàm thế
dt = 0.05;               % bước thời gian
T = 500;                  % tổng thời gian mô phỏng
n_steps = T/dt;          % số bước mô phỏng

%% ========================================================================
%  2. TẠO MA TRẬN Q (CƠ SỞ TRỰC CHUẨN CHO MẶT PHẲNG BÙ)
%% ========================================================================
fprintf('Tạo ma trận Q...\n');
Q = create_Q(n_drones);

%% ========================================================================
%  3. KHỞI TẠO VỊ TRÍ BAN ĐẦU
%% ========================================================================
% Tạo vị trí ngẫu nhiên trong không gian [-15,15]^3
P = zeros(dim, n_drones, n_steps);
P(:,:,1) = 30 * rand(dim, n_drones) - 15;

fprintf('Vị trí ban đầu:\n');
disp(P(:,:,1));

%% ========================================================================
%  4. VÒNG LẶP MÔ PHỎNG CHÍNH
%% ========================================================================
fprintf('\nBắt đầu mô phỏng %d bước (T = %.1f s)...\n', n_steps, T);

lambda2_history = zeros(n_steps, 1);
detM_history = zeros(n_steps, 1);

for k = 1:n_steps-1
    p_current = P(:,:,k);
    
    % 4.1. Tính lực điều khiển giữ liên kết
    [u, detM, lambda2] = connectivity_control(p_current, delta, omega, Q, alpha);
    
    % 4.2. Lưu lịch sử
    detM_history(k) = detM;
    lambda2_history(k) = lambda2;
    
    % 4.3. Cập nhật vị trí (Euler)
    P(:,:,k+1) = p_current + u * dt;
    
    % 4.4. Hiển thị tiến độ
    if mod(k, round(n_steps/10)) == 0
        fprintf('  t = %.1f s, det(M) = %.4f, λ2 = %.4f\n', k*dt, detM, lambda2);
    end
end
fprintf('Mô phỏng hoàn tất!\n');

%% ========================================================================
%  5. VẼ KẾT QUẢ
%% ========================================================================

% 5.1. Vẽ quỹ đạo 3D
figure('Position', [100, 100, 1200, 800]);

% Quỹ đạo 3D
subplot(2,3,[1,2,4,5]);
colors = lines(n_drones);
hold on;

for i = 1:n_drones
    plot3(squeeze(P(1,i,:)), squeeze(P(2,i,:)), squeeze(P(3,i,:)), ...
        'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Drone %d', i));
end

% Đánh dấu vị trí đầu và cuối
for i = 1:n_drones
    % Vị trí đầu (hình tròn)
    plot3(P(1,i,1), P(2,i,1), P(3,i,1), 'o', ...
        'Color', colors(i,:), 'MarkerSize', 8, 'MarkerFaceColor', 'w', ...
        'MarkerEdgeColor', colors(i,:));
    
    % Vị trí cuối (hình vuông)
    plot3(P(1,i,end), P(2,i,end), P(3,i,end), 's', ...
        'Color', colors(i,:), 'MarkerSize', 10, 'MarkerFaceColor', colors(i,:));
end

xlabel('x'); ylabel('y'); zlabel('z');
title('Quỹ đạo drone với luật giữ liên kết');
legend('Location', 'best');
grid on; axis equal;
view(45, 30);

% 5.2. Vẽ det(M) theo thời gian
subplot(2,3,3);
time = (0:n_steps-1) * dt;
plot(time, detM_history, 'b-', 'LineWidth', 2);
xlabel('Thời gian (s)'); ylabel('det(M)');
title('det(M) theo thời gian');
grid on;
ylim([0, max(detM_history)*1.1]);

% 5.3. Vẽ λ2 theo thời gian
subplot(2,3,6);
plot(time, lambda2_history, 'r-', 'LineWidth', 2);
xlabel('Thời gian (s)'); ylabel('\lambda_2');
title('\lambda_2 (chỉ số liên thông)');
grid on;
ylim([0, max(lambda2_history)*1.1]);

sgtitle('Mô phỏng luật giữ liên kết', 'FontSize', 14);

%% ========================================================================
%  6. VẼ KHÔNG GIAN GIAO TIẾP TẠI THỜI ĐIỂM CUỐI
%% ========================================================================
figure('Position', [100, 100, 800, 600]);

% Tính ma trận kề tại thời điểm cuối
A_final = adjacency_sigmoid(P(:,:,end), delta, omega);

% Vẽ đồ thị giao tiếp
G = graph(A_final, {'D1','D2','D3','D4'});
plot(G, 'Layout', 'force', 'NodeColor', 'r', 'MarkerSize', 10, ...
    'EdgeColor', 'b', 'LineWidth', 2);
title(sprintf('Đồ thị giao tiếp tại t = %.1f s', T));

%% ========================================================================
%  7. FUNCTIONS (ĐẶT Ở CUỐI FILE)
%% ========================================================================

%% Hàm tạo ma trận Q
function Q = create_Q(n)
    % Tạo n-1 vector ngẫu nhiên
    Q = randn(n, n-1);
    
    % Gram-Schmidt để trực chuẩn hóa và vuông góc với 1_n
    for i = 1:n-1
        % Trực giao với 1_n
        Q(:,i) = Q(:,i) - (1/n) * (ones(1,n) * Q(:,i)) * ones(n,1);
        
        % Trực giao với các vector trước
        for j = 1:i-1
            Q(:,i) = Q(:,i) - (Q(:,j)' * Q(:,i)) * Q(:,j);
        end
        
        % Chuẩn hóa
        norm_i = norm(Q(:,i));
        if norm_i > 1e-10
            Q(:,i) = Q(:,i) / norm_i;
        else
            % Nếu vector gần 0, tạo lại
            Q(:,i) = randn(n,1);
            Q(:,i) = Q(:,i) - (1/n) * (ones(1,n) * Q(:,i)) * ones(n,1);
            Q(:,i) = Q(:,i) / norm(Q(:,i));
        end
    end
end

%% Hàm tạo ma trận kề sigmoid
function A = adjacency_sigmoid(P, delta, omega)
    [n_dim, n] = size(P);
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

%% Hàm tạo ma trận Laplace
function L = laplacian_from_adjacency(A)
    D = diag(sum(A, 2));
    L = D - A;
end

%% Hàm tính M = Q' * L * Q
function M = compute_M(P, delta, omega, Q)
    A = adjacency_sigmoid(P, delta, omega);
    L = laplacian_from_adjacency(A);
    M = Q' * L * Q;
end

%% Hàm tính đạo hàm số của M theo vị trí
function dM = numerical_dM_dp(P, i_coord, delta, omega, Q, eps)
    % i_coord: [drone_index, coordinate_index] (vd: [2,1] là x của drone 2)
    % eps: bước sai phân
    % dM: đạo hàm của M theo tọa độ đó
    
    drone_i = i_coord(1);
    coord_d = i_coord(2);
    
    % Tính M tại p
    M0 = compute_M(P, delta, omega, Q);
    
    % Tính M tại p + eps
    P_plus = P;
    P_plus(coord_d, drone_i) = P_plus(coord_d, drone_i) + eps;
    M_plus = compute_M(P_plus, delta, omega, Q);
    
    % Sai phân trung tâm (chính xác hơn)
    P_minus = P;
    P_minus(coord_d, drone_i) = P_minus(coord_d, drone_i) - eps;
    M_minus = compute_M(P_minus, delta, omega, Q);
    
    dM = (M_plus - M_minus) / (2*eps);
end

%% Hàm tính luật điều khiển giữ liên kết
function [u, detM, lambda2] = connectivity_control(P, delta, omega, Q, alpha)
    [n_dim, n] = size(P);
    
    % Tính M
    M = compute_M(P, delta, omega, Q);
    
    % Tính định thức
    detM = det(M);
    if detM < 1e-6
        detM = 1e-6;  % tránh chia cho 0
    end
    
    % Tính λ2 (chỉ số liên thông)
    A = adjacency_sigmoid(P, delta, omega);
    L = laplacian_from_adjacency(A);
    e = eig(L);
    e = sort(e);
    if length(e) >= 2
        lambda2 = e(2);
    else
        lambda2 = 0;
    end
    
    % Tính M^{-1}
    invM = inv(M);
    
    % Tính gradient bằng sai phân số
    u = zeros(n_dim, n);
    eps = 1e-4;  % bước sai phân
    
    for i = 1:n
        for d = 1:n_dim
            % Tính đạo hàm của M theo tọa độ d của drone i
            dM = numerical_dM_dp(P, [i, d], delta, omega, Q, eps);
            
            % trace(M^{-1} * dM)
            trace_val = trace(invM * dM);
            
            % Công thức (7.10)
            u(d,i) = (alpha / detM^alpha) * trace_val;
        end
    end
    
    % Giới hạn lực điều khiển (tránh quá lớn)
    max_u = 10;
    u = max(min(u, max_u), -max_u);
end