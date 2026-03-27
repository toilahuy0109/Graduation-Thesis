%% MÔ PHỎNG 5 DRONE - TRÁNH VA CHẠM VỚI HÀM β TỪ SLIDE
clear; clc; close all;

%% ========================================================================
%  1. THAM SỐ
%% ========================================================================
n_drones = 5;
dim = 2;
dt = 0.02;
T = 60;
t = 0:dt:T;
n_steps = length(t);

% Quỹ đạo hình tròn cho các drone (mỗi drone bay trên 1 quỹ đạo riêng)
R = 12;
omega = 0.15;
center = [15; 15];

% Pha ban đầu cho 5 drone (để chúng không trùng nhau)
phases = [0, 0.5, 1.0, 1.5, 2.0];  % rad

% Ngưỡng va chạm d
d_collision = 3.0;
mu = (1 + d_collision^4) / d_collision^4;

% Tham số điều khiển
alpha = 2.0;            % tham số hàm thế
k_avoid = 15.0;         % hệ số tránh va chạm (tăng cho 5 drone)
k_track = 0.4;          % bám quỹ đạo (giảm để ưu tiên tránh)
k_damping = 0.8;        % damping
k_repulsion = 10.0;     % lực đẩy bổ sung khi quá gần

% Nhiễu - đẩy các drone lại gần nhau
disturbance_time = 20;
disturbance_strength = 25;
disturbance_duration = 0.5;

%% ========================================================================
%  2. ĐỊNH NGHĨA HÀM βij TỪ SLIDE
%% ========================================================================
function beta = beta_slide(dist, d, mu)
    % Hàm β cho tránh va chạm theo công thức slide
    % dist: khoảng cách hiện tại
    % d: ngưỡng va chạm
    % mu: tham số đã tính từ d
    
    if (dist^2 - d^2) >= 0
        rho = 0;
    else
        rho = 1;
    end
    
    diff_sq = (dist^2 - d^2)^2;
    beta = (1 - mu * diff_sq / (1 + diff_sq))^rho;
end

% Đạo hàm số của β theo dist
function dbeta = dbeta_ddist_slide(dist, d, mu, eps)
    if dist < eps
        dbeta = 0;
    else
        beta_plus = beta_slide(dist + eps, d, mu);
        beta_minus = beta_slide(dist - eps, d, mu);
        dbeta = (beta_plus - beta_minus) / (2 * eps);
    end
end

%% ========================================================================
%  3. TẠO ĐỒ THỊ KẾT NỐI ĐẦY ĐỦ (COMPLETE GRAPH)
%% ========================================================================
% Với 5 drone, đồ thị đầy đủ có 10 cạnh
edges = [];
for i = 1:n_drones
    for j = i+1:n_drones
        edges = [edges; i, j];
    end
end
n_edges = size(edges, 1);
fprintf('Số cạnh: %d (đồ thị đầy đủ)\n', n_edges);

%% ========================================================================
%  4. KHỞI TẠO VỊ TRÍ
%% ========================================================================
P = zeros(dim, n_drones, n_steps);
V = zeros(dim, n_drones, n_steps);

% Vị trí ban đầu (trên quỹ đạo tròn với các pha khác nhau)
for i = 1:n_drones
    angle = phases(i);
    P(:,i,1) = center + R * [cos(angle); sin(angle)];
end

% Thêm nhiễu nhỏ ban đầu để không đối xứng hoàn hảo
P(:,:,1) = P(:,:,1) + 0.5 * randn(dim, n_drones);
V(:,:,1) = zeros(dim, n_drones);

fprintf('=== MÔ PHỎNG 5 DRONE TRÁNH VA CHẠM ===\n');
fprintf('Ngưỡng va chạm d = %.1f m\n', d_collision);
fprintf('Số cạnh tương tác: %d\n', n_edges);

%% ========================================================================
%  5. VÒNG LẶP MÔ PHỎNG
%% ========================================================================
dist_min_history = zeros(n_steps, 1);
beta_min_history = zeros(n_steps, 1);
Phi_history = zeros(n_steps, 1);
force_avg_history = zeros(n_steps, 1);
danger_duration = 0;

eps_deriv = 1e-4;

for k = 1:n_steps-1
    p_curr = P(:,:,k);
    v_curr = V(:,:,k);
    
    % Quỹ đạo tham chiếu cho từng drone
    p_ref = zeros(dim, n_drones);
    for i = 1:n_drones
        p_ref(:,i) = center + R * [cos(omega*t(k) + phases(i)); sin(omega*t(k) + phases(i))];
    end
    
    u = zeros(dim, n_drones);
    
    % Tính tích các β và tìm khoảng cách nhỏ nhất
    beta_product = 1.0;
    min_dist = inf;
    
    for e = 1:n_edges
        i = edges(e,1);
        j = edges(e,2);
        dij = norm(p_curr(:,i) - p_curr(:,j));
        min_dist = min(min_dist, dij);
        
        beta_val = beta_slide(dij, d_collision, mu);
        beta_product = beta_product * beta_val;
    end
    
    % Hàm thế Φ = 1/β^α
    Phi = 1 / (beta_product^alpha);
    dPhi_dbeta = -alpha * beta_product^(-alpha-1);
    
    % Tính lực cho từng drone
    total_force = 0;
    
    for i = 1:n_drones
        % Bám quỹ đạo
        u_track = -k_track * (p_curr(:,i) - p_ref(:,i));
        
        % Damping
        u_damp = -k_damping * v_curr(:,i);
        
        % Tránh va chạm với tất cả drone khác
        u_avoid = zeros(dim, 1);
        u_repulsion = zeros(dim, 1);
        
        for j = 1:n_drones
            if j == i, continue; end
            
            p_ij = p_curr(:,i) - p_curr(:,j);
            dij_ij = norm(p_ij);
            
            if dij_ij > 1e-6
                % Vector đơn vị
                direction = p_ij / dij_ij;
                
                % Lực từ hàm thế (gradient descent)
                dbeta_dd = dbeta_ddist_slide(dij_ij, d_collision, mu, eps_deriv);
                grad = dPhi_dbeta * dbeta_dd * direction;
                u_avoid = u_avoid - k_avoid * grad;
                
                % Lực đẩy bổ sung khi quá gần (d < d_collision/2)
                if dij_ij < d_collision/2
                    repulsion_force = k_repulsion * (1/dij_ij - 2/d_collision);
                    u_repulsion = u_repulsion + repulsion_force * direction;
                end
                
                total_force = total_force + norm(grad);
            end
        end
        
        u(:,i) = u_track + u_damp + u_avoid + u_repulsion;
    end
    
    % Nhiễu - đẩy các drone lại gần nhau
    if t(k) >= disturbance_time && t(k) < disturbance_time + disturbance_duration
        for i = 1:n_drones
            % Đẩy về phía trung tâm
            to_center = center - p_curr(:,i);
            if norm(to_center) > 0
                u(:,i) = u(:,i) + disturbance_strength * (to_center / norm(to_center));
            end
        end
    end
    
    % Giới hạn lực
    max_u = 40;
    u = max(min(u, max_u), -max_u);
    
    % Cập nhật
    V(:,:,k+1) = V(:,:,k) + u * dt;
    P(:,:,k+1) = P(:,:,k) + V(:,:,k) * dt;
    
    % Lưu lịch sử
    dist_min_history(k+1) = min_dist;
    beta_min_history(k+1) = beta_slide(min_dist, d_collision, mu);
    Phi_history(k+1) = Phi;
    force_avg_history(k+1) = total_force / n_edges;
    
    if min_dist < d_collision
        danger_duration = danger_duration + dt;
    end
    
    if mod(k, 500) == 0
        fprintf('t = %.1f s, d_min = %.3f, β_min = %.4f, Φ = %.2f\n', ...
            t(k), min_dist, beta_min_history(k), Phi);
    end
end

fprintf('Mô phỏng hoàn tất!\n');
fprintf('Khoảng cách nhỏ nhất: %.3f m\n', min(dist_min_history));
fprintf('Thời gian trong vùng nguy hiểm: %.2f s\n', danger_duration);

%% ========================================================================
%  6. VẼ KẾT QUẢ
%% ========================================================================
figure('Position', [50, 50, 1600, 900]);

% 6.1. Hàm β mẫu
subplot(2,4,1);
d_test = linspace(0, 2*d_collision, 200);
beta_test = zeros(size(d_test));
for i = 1:length(d_test)
    beta_test(i) = beta_slide(d_test(i), d_collision, mu);
end
plot(d_test, beta_test, 'b-', 'LineWidth', 2);
hold on;
xline(d_collision, 'r--', 'd = 3m', 'LineWidth', 2);
xlabel('Khoảng cách (m)'); ylabel('β');
title('Hàm β(d) từ slide');
grid on;
ylim([-0.1, 1.1]);

% 6.2. Khoảng cách nhỏ nhất
subplot(2,4,2);
plot(t, dist_min_history, 'b-', 'LineWidth', 2);
hold on;
yline(d_collision, 'r--', 'Ngưỡng', 'LineWidth', 2);
xline(disturbance_time, 'm--', 'Nhiễu', 'LineWidth', 2);
xlabel('Thời gian (s)'); ylabel('d_min (m)');
title('Khoảng cách nhỏ nhất');
grid on;

% 6.3. β_min theo thời gian
subplot(2,4,3);
plot(t, beta_min_history, 'g-', 'LineWidth', 2);
hold on;
yline(0.5, 'k--', 'β=0.5');
xline(disturbance_time, 'm--', 'Nhiễu', 'LineWidth', 2);
xlabel('Thời gian (s)'); ylabel('β_{min}');
title('β nhỏ nhất');
grid on;
ylim([-0.1, 1.1]);

% 6.4. Hàm thế Φ
subplot(2,4,4);
semilogy(t, Phi_history, 'r-', 'LineWidth', 2);
hold on;
xline(disturbance_time, 'm--', 'Nhiễu', 'LineWidth', 2);
xlabel('Thời gian (s)'); ylabel('Φ');
title('Hàm thế Φ (log scale)');
grid on;

% 6.5. Lực trung bình
subplot(2,4,5);
plot(t, force_avg_history, 'm-', 'LineWidth', 2);
hold on;
xline(disturbance_time, 'm--', 'Nhiễu', 'LineWidth', 2);
xlabel('Thời gian (s)'); ylabel('Lực TB');
title('Lực tránh trung bình');
grid on;

% 6.6. Phân bố khoảng cách
subplot(2,4,6);
histogram(dist_min_history, 50);
xlabel('Khoảng cách (m)'); ylabel('Tần số');
title('Phân bố khoảng cách nhỏ nhất');
grid on;

% 6.7. Quỹ đạo 5 drone
subplot(2,4,[7,8]);
colors = lines(n_drones);
hold on; grid on; box on;

% Vẽ quỹ đạo tham chiếu
theta = linspace(0, 2*pi, 100);
plot(center(1) + R*cos(theta), center(2) + R*sin(theta), 'k--', 'LineWidth', 1);

% Vẽ quỹ đạo thực tế
for i = 1:n_drones
    plot(squeeze(P(1,i,:)), squeeze(P(2,i,:)), 'Color', colors(i,:), ...
        'LineWidth', 1.5, 'DisplayName', sprintf('Drone %d', i));
end

% Vị trí cuối
for i = 1:n_drones
    plot(P(1,i,end), P(2,i,end), 'o', 'Color', colors(i,:), ...
        'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
end

xlabel('x (m)'); ylabel('y (m)');
title('Quỹ đạo 5 drone');
legend('Location', 'best');
axis equal;
xlim([-5, 35]); ylim([-5, 35]);

sgtitle('5 Drone - Tránh va chạm với hàm β từ slide', ...
    'FontSize', 16, 'FontWeight', 'bold');