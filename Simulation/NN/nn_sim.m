%% MÔ PHỎNG 5 DRONE - GIỮ KHOẢNG CÁCH + GIỮ LIÊN KẾT (XẤP XỈ NN)
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

% Tham số điều khiển formation (distance-based)
alpha_form = 1.0;
beta_damp = 5.0;
beta_leader = 10.0;
gamma_leader = 2.0;

% Quỹ đạo leader (drone 1)
R = 15;
h = 8;
vz = 0.5;
omega = 0.2;

% Tham số giữ liên kết
delta_comm = 12;        % ngưỡng giao tiếp (m)
omega_sig = 1;          % độ dốc sigmoid
alpha_phi = 2;          % tham số hàm thế
k_conn = 0.5;           % hệ số lực giữ liên kết (cho exact)

% Tham số mạng nơ-ron cho xấp xỉ
n_input = dim * n_drones;   % 15
n_hidden = 32;              % số nơ-ron lớp ẩn
learning_rate = 0.005;      % learning rate
update_freq = 50;           % cập nhật mỗi 50 bước
eps_fd = 1e-4;              % bước sai phân số

% Lựa chọn: 'exact' hoặc 'nn'
control_mode = 'nn';  % 'exact' hoặc 'nn'

%% ========================================================================
%  2. ĐỘI HÌNH MONG MUỐN
%% ========================================================================
L = 6;
H = 8;

p_rel_star = zeros(dim, n_drones);
p_rel_star(:,1) = [0; 0; 0];           % leader
p_rel_star(:,2) = [L; L; -H];          % follower 2
p_rel_star(:,3) = [L; -L; -H];         % follower 3
p_rel_star(:,4) = [-L; L; -H];         % follower 4
p_rel_star(:,5) = [-L; -L; -H];        % follower 5

%% ========================================================================
%  3. ĐỒ THỊ CỨNG 9 CẠNH
%% ========================================================================
edges = [1 2; 1 3; 1 4; 1 5; 2 3; 2 4; 3 4; 3 5; 4 5];
m = size(edges, 1);

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
%  4. TẠO MA TRẬN Q CHO GIỮ LIÊN KẾT
%% ========================================================================
Q = create_Q_matrix(n_drones);

%% ========================================================================
%  5. KHỞI TẠO MẠNG NƠ-RON
%% ========================================================================
if strcmp(control_mode, 'nn')
    nn = OneLayerNN(n_input, n_hidden, learning_rate);
    fprintf('Sử dụng mạng nơ-ron để xấp xỉ gradient\n');
else
    fprintf('Sử dụng gradient chính xác\n');
end

%% ========================================================================
%  6. KHỞI TẠO VỊ TRÍ
%% ========================================================================
P = zeros(dim, n_drones, n_steps);
V = zeros(dim, n_drones, n_steps);

% Leader bắt đầu tại gốc
P(:,1,1) = [0; 0; h];

% Followers bắt đầu lệch
for i = 2:n_drones
    P(:,i,1) = P(:,1,1) + p_rel_star(:,i) + 2*randn(3,1);
end
V(:,:,1) = zeros(dim, n_drones);

%% ========================================================================
%  7. KHỞI TẠO LỊCH SỬ
%% ========================================================================
formation_error = zeros(n_steps, 1);
leader_error = zeros(n_steps, 1);
lambda2_history = zeros(n_steps, 1);
nn_loss_history = zeros(floor(n_steps/update_freq), 1);
grad_error_history = zeros(floor(n_steps/update_freq), 1);
loss_idx = 1;

fprintf('=== MÔ PHỎNG %d DRONE ===\n', n_drones);
fprintf('Chế độ điều khiển: %s\n', control_mode);
fprintf('Ngưỡng giao tiếp: %.1f m\n', delta_comm);

%% ========================================================================
%  8. VÒNG LẶP MÔ PHỎNG CHÍNH
%% ========================================================================
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
    u = zeros(dim, n_drones);
    u(:,i) = u_leader;
    leader_error(k+1) = norm(p_curr(:,i) - p_ref);
    
    % =====================================================================
    %  TÍNH LỰC GIỮ LIÊN KẾT
    % =====================================================================
    if strcmp(control_mode, 'nn')
        % Dùng mạng nơ-ron
        p_flat = p_curr(:);
        u_conn_nn = nn.forward(p_flat);
        u_conn = reshape(u_conn_nn, dim, n_drones);
        
        % Cập nhật mạng (thỉnh thoảng)
        if mod(k, update_freq) == 0
            % Tính gradient chính xác bằng sai phân số
            u_exact = compute_exact_gradient(p_curr, Q, delta_comm, omega_sig, alpha_phi, eps_fd);
            u_exact_flat = u_exact(:);
            
            % Cập nhật mạng
            loss = nn.update(p_flat, u_exact_flat);
            nn_loss_history(loss_idx) = loss;
            
            % Tính sai số giữa NN và exact
            grad_error = norm(u_conn_nn - u_exact_flat) / norm(u_exact_flat);
            grad_error_history(loss_idx) = grad_error;
            
            loss_idx = loss_idx + 1;
            
            if mod(k, 500) == 0
                fprintf('t = %.1f s, loss = %.6f, grad_error = %.4f\n', t(k), loss, grad_error);
            end
        end
    else
        % Dùng gradient chính xác (chậm)
        u_conn = compute_exact_gradient(p_curr, Q, delta_comm, omega_sig, alpha_phi, eps_fd);
        u_conn = k_conn * u_conn;  % Thêm hệ số
    end
    
    % =====================================================================
    %  FOLLOWERS (Drone 2-5) - LỰC GIỮ KHOẢNG CÁCH
    % =====================================================================
    for i = 2:n_drones
        u_form = zeros(dim, 1);
        
        for j = neighbors{i}
            p_ij = p_curr(:,j) - p_curr(:,i);
            dij = norm(p_ij);
            
            if dij > 1e-6
                % Tìm chỉ số cạnh
                e_idx = 0;
                for e = 1:m
                    if (edges(e,1) == i && edges(e,2) == j) || ...
                       (edges(e,1) == j && edges(e,2) == i)
                        e_idx = e;
                        break;
                    end
                end
                
                if e_idx > 0
                    u_form = u_form + alpha_form * (dij^2 - d_star(e_idx)^2) * p_ij;
                end
            end
        end
        
        % Damping
        u_damp = -beta_damp * v_curr(:,i);
        
        % Tổng hợp lực formation + damping
        u(:,i) = u_form + u_damp;
        
        % Cộng thêm lực giữ liên kết
        if strcmp(control_mode, 'nn')
            u(:,i) = u(:,i) + 0.5 * u_conn(:,i);  % Hệ số nhỏ cho NN
        else
            u(:,i) = u(:,i) + k_conn * u_conn(:,i);
        end
    end
    
    % Giới hạn lực
    max_u = 15;
    u = max(min(u, max_u), -max_u);
    
    % Cập nhật vị trí
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
    
    % Tính chỉ số liên thông λ₂
    A = compute_adjacency(p_curr, delta_comm, omega_sig);
    L = diag(sum(A,2)) - A;
    e = eig(L);
    e = sort(e);
    lambda2_history(k+1) = e(2);
end

fprintf('Mô phỏng hoàn tất!\n');
fprintf('Sai số cuối: leader = %.4f m, formation = %.4f m\n', ...
    leader_error(end), formation_error(end));

%% ========================================================================
%  9. VẼ KẾT QUẢ
%% ========================================================================
figure('Position', [50, 50, 1600, 900]);

% 9.1. Quỹ đạo 3D
subplot(2,4,[1,2,5,6]);
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
title(['Quỹ đạo drone - ', control_mode, ' control']);
legend('Location', 'best');
view(45,30); axis equal;

% 9.2. Sai số leader
subplot(2,4,3);
plot(t, leader_error, 'r-', 'LineWidth', 2);
xlabel('Thời gian (s)'); ylabel('Sai số (m)');
title('Sai số leader');
grid on;

% 9.3. Sai số đội hình
subplot(2,4,4);
plot(t, formation_error, 'b-', 'LineWidth', 2);
xlabel('Thời gian (s)'); ylabel('Sai số RMS (m)');
title('Sai số đội hình');
grid on;

% 9.4. Chỉ số liên thông
subplot(2,4,7);
plot(t, lambda2_history, 'g-', 'LineWidth', 2);
hold on;
yline(0, 'r--', 'Nguy hiểm');
xlabel('Thời gian (s)'); ylabel('λ₂');
title('Chỉ số liên thông');
grid on;

% 9.5. Loss và sai số gradient (nếu dùng NN)
if strcmp(control_mode, 'nn')
    subplot(2,4,8);
    update_times = (1:loss_idx-1) * update_freq * dt;
    
    yyaxis left;
    semilogy(update_times, nn_loss_history(1:loss_idx-1), 'b-', 'LineWidth', 2);
    ylabel('Loss');
    
    yyaxis right;
    plot(update_times, grad_error_history(1:loss_idx-1), 'r-', 'LineWidth', 2);
    ylabel('Gradient error');
    
    xlabel('Thời gian (s)');
    title('Huấn luyện mạng');
    legend('Loss', 'Grad error');
    grid on;
end

sgtitle(['Formation + Connectivity - ', control_mode], 'FontSize', 14, 'FontWeight', 'bold');

%% ========================================================================
%  10. SO SÁNH VỚI EXACT (NẾU DÙNG NN)
%% ========================================================================
if strcmp(control_mode, 'nn')
    % Chạy lại với exact để so sánh (nếu muốn)
    % Có thể lưu kết quả và so sánh riêng
end

%% ========================================================================
%  ĐỊNH NGHĨA CÁC HÀM
%% ========================================================================

function Q = create_Q_matrix(n)
    Q = randn(n, n-1);
    for i = 1:n-1
        Q(:,i) = Q(:,i) - (1/n) * (ones(1,n) * Q(:,i)) * ones(n,1);
        for j = 1:i-1
            Q(:,i) = Q(:,i) - (Q(:,j)' * Q(:,i)) * Q(:,j);
        end
        Q(:,i) = Q(:,i) / norm(Q(:,i));
    end
end

function A = compute_adjacency(P, delta, omega)
    n = size(P,2);
    A = zeros(n);
    for i = 1:n
        for j = i+1:n
            d = norm(P(:,i) - P(:,j));
            a = 1 / (1 + exp(-omega * (delta - d)));
            A(i,j) = a;
            A(j,i) = a;
        end
    end
end

function Phi = compute_potential(P, Q, delta, omega, alpha)
    n = size(P,2);
    A = compute_adjacency(P, delta, omega);
    L = diag(sum(A,2)) - A;
    M = Q' * L * Q;
    detM = det(M);
    if detM < 1e-6
        detM = 1e-6;
    end
    Phi = 1 / (detM^alpha);
end

function u_conn = compute_exact_gradient(P, Q, delta, omega, alpha, eps)
    [dim, n] = size(P);
    u_conn = zeros(dim, n);
    Phi0 = compute_potential(P, Q, delta, omega, alpha);
    
    for i = 1:n
        for d = 1:dim
            P_plus = P;
            P_plus(d,i) = P_plus(d,i) + eps;
            Phi_plus = compute_potential(P_plus, Q, delta, omega, alpha);
            
            P_minus = P;
            P_minus(d,i) = P_minus(d,i) - eps;
            Phi_minus = compute_potential(P_minus, Q, delta, omega, alpha);
            
            u_conn(d,i) = -(Phi_plus - Phi_minus) / (2*eps);
        end
    end
end