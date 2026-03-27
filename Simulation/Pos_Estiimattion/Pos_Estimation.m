%% LUẬT CẬP NHẬT ĐỘNG HỌC CHO ĐỊNH VỊ (dạng vi phân)
clear; clc; close all;

%% ========================================================================
%  1. THAM SỐ
%% ========================================================================
n_drones = 5;
dim = 2;
dt = 0.02;
T = 30;
t = 0:dt:T;
n_steps = length(t);

% Đồ thị kết nối
edges = [1 2; 1 3; 1 4; 1 5; 2 3; 3 4; 4 5];

% Quỹ đạo leader (drone 1)
R = 20; omega = 0.2;
p_leader_true = [R*cos(omega*t); R*sin(omega*t)];

% Đội hình thật
L = 6;
offset_true = [0, 0; L, 0; -L, 0; 0, L; 0, -L]';

P_true = zeros(dim, n_drones, n_steps);
for k = 1:n_steps
    for i = 1:n_drones
        P_true(:,i,k) = p_leader_true(:,k) + offset_true(:,i);
    end
end

% Nhiễu đo lường
gps_noise = 0.5;
range_noise = 0.1;

% Tham số điều khiển
alpha = 0.5;   % Hệ số gradient
beta = 1.0;    % Hệ số neo về GPS

%% ========================================================================
%  2. ĐO VỊ TRÍ TƯƠNG ĐỐI (có nhiễu)
%% ========================================================================
z_meas = zeros(dim, n_drones, n_drones, n_steps);

for k = 1:n_steps
    for e = 1:size(edges,1)
        i = edges(e,1);
        j = edges(e,2);
        
        z_true = P_true(:,j,k) - P_true(:,i,k);
        z_meas(:,j,i,k) = z_true + range_noise * randn(dim,1);
        z_meas(:,i,j,k) = -z_meas(:,j,i,k);
    end
end

%% ========================================================================
%  3. KHỞI TẠO BỘ ĐỊNH VỊ
%% ========================================================================
P_est = zeros(dim, n_drones, n_steps);
P_dot = zeros(dim, n_drones, n_steps);

% Drone 1: GPS (có nhiễu)
P_est(:,1,1) = P_true(:,1,1) + gps_noise * randn(dim,1);

% Các follower khởi tạo ngẫu nhiên
for i = 2:n_drones
    P_est(:,i,1) = 20 * randn(dim,1);
end

%% ========================================================================
%  4. VÒNG LẶP CẬP NHẬT THEO LUẬT VI PHÂN
%% ========================================================================
fprintf('=== LUẬT CẬP NHẬT ĐỘNG HỌC ===\n');
fprintf('dP_i/dt = -α Σ(ẑ_ij - z_ij) - β (P_i - P_1)\n\n');

for k = 1:n_steps-1
    % =====================================================================
    %  CẬP NHẬT GPS CỦA LEADER
    % =====================================================================
    P_est(:,1,k+1) = P_true(:,1,k+1) + gps_noise * randn(dim,1);
    
    % =====================================================================
    %  TÍNH ĐẠO HÀM THEO LUẬT (8.1) + NEO
    % =====================================================================
    for i = 2:n_drones
        % Thành phần gradient (8.1)
        grad_term = zeros(dim,1);
        for e = 1:size(edges,1)
            if edges(e,1) == i
                j = edges(e,2);
                z_est = P_est(:,j,k) - P_est(:,i,k);
                grad_term = grad_term - 10*(z_est - z_meas(:,j,i,k));
            elseif edges(e,2) == i
                j = edges(e,1);
                z_est = P_est(:,j,k) - P_est(:,i,k);
                grad_term = grad_term - 10*(z_est - z_meas(:,j,i,k));
            end
        end
        
        % Thành phần neo về leader (nếu có kết nối trực tiếp)
        anchor_term = zeros(dim,1);
        for e = 1:size(edges,1)
            if (edges(e,1) == 1 && edges(e,2) == i) || (edges(e,1) == i && edges(e,2) == 1)
                z_1i_est = P_est(:,i,k) - P_est(:,1,k);
                anchor_term = anchor_term + (z_1i_est - z_meas(:,i,1,k));
            end
        end
        
        % Luật cập nhật vi phân
        P_dot(:,i,k) = -alpha * grad_term - beta * anchor_term;
        
        % Euler integration
        P_est(:,i,k+1) = P_est(:,i,k) + P_dot(:,i,k) * dt;
    end
end

%% ========================================================================
%  5. KẾT QUẢ
%% ========================================================================
fprintf('KẾT QUẢ CUỐI:\n');
for i = 1:n_drones
    error_i = norm(P_est(:,i,end) - P_true(:,i,end));
    fprintf('Drone %d: sai số = %.3f m\n', i, error_i);
end

%% ========================================================================
%  6. VẼ KẾT QUẢ
%% ========================================================================
figure('Position', [100, 100, 1400, 800]);

% 6.1. Quỹ đạo
subplot(2,2,1);
colors = lines(n_drones);
hold on; grid on; box on;

for i = 1:n_drones
    plot(squeeze(P_true(1,i,:)), squeeze(P_true(2,i,:)), ...
        'Color', colors(i,:), 'LineWidth', 1.5, 'LineStyle', '-');
    plot(squeeze(P_est(1,i,:)), squeeze(P_est(2,i,:)), ...
        'Color', colors(i,:), 'LineWidth', 1.5, 'LineStyle', '--');
end
xlabel('x'); ylabel('y'); title('Quỹ đạo (thật: liền, ước lượng: đứt)');
axis equal;

% 6.2. Sai số theo thời gian
subplot(2,2,2);
hold on; grid on;
for i = 1:n_drones
    error_i = squeeze(sqrt(sum((P_est(:,i,:) - P_true(:,i,:)).^2, 1)));
    plot(t, error_i, 'Color', colors(i,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Drone %d', i));
end
xlabel('Thời gian (s)'); ylabel('Sai số (m)');
title('Sai số ước lượng');
legend('Location', 'best');

% 6.3. Lực cập nhật
subplot(2,2,3);
plot(t, squeeze(P_dot(1,2,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(t, squeeze(P_dot(2,2,:)), 'r-', 'LineWidth', 1.5);
xlabel('Thời gian (s)'); ylabel('dP/dt (m/s)');
title('Lực cập nhật của drone 2');
legend('dx/dt', 'dy/dt');
grid on;

% 6.4. Đồ thị kết nối
subplot(2,2,4);
A = zeros(n_drones);
for e = 1:size(edges,1)
    i = edges(e,1);
    j = edges(e,2);
    A(i,j) = 1;
    A(j,i) = 1;
end
G = graph(A);
plot(G, 'Layout', 'force', 'NodeColor', 'r', 'MarkerSize', 15, ...
    'EdgeColor', 'b', 'LineWidth', 2);
title('Đồ thị kết nối');
for i = 1:n_drones
    text(G.Nodes.XData(i), G.Nodes.YData(i), num2str(i), ...
        'FontSize', 12, 'FontWeight', 'bold', 'Color', 'white');
end

sgtitle('Luật cập nhật động học: dP_i/dt = -α Σ(ẑ_ij - z_ij) - β (P_i - P_1)');