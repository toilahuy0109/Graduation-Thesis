
clear; clc; close all;

%% ========================================================================
%  1. THAM SỐ
%% ========================================================================
n_drones = 5;
dim = 3;
dt = 0.01;
T = 60;
t = 0:dt:T;
n_steps = length(t);

% Quỹ đạo leader
R = 15; omega = 0.2; h = 8; vz = 0.5;
p_leader_true = [R*cos(omega*t); R*sin(omega*t); h + vz*t];

% Đội hình
L = 6; H = 8;
offset = [0, 0; L, L; L, -L; -L, L; -L, -L]';
p_rel_star = [offset; zeros(1,5)];

% Vị trí thật
P_true = zeros(dim, n_drones, n_steps);
for k = 1:n_steps
    for i = 1:n_drones
        P_true(:,i,k) = p_leader_true(:,k) + p_rel_star(:,i);
    end
end

% Nhiễu ban đầu
for i = 2:n_drones
    P_true(:,i,1) = P_true(:,i,1) + 5*randn(3,1);
end

%% ========================================================================
%  2. SWITCHING TOPOLOGY (periodic jointly connected)
%% ========================================================================
edges1 = [1 2; 1 3; 2 3];           % Topo 1: leader + 2,3
edges2 = [1 4; 1 5; 4 5];           % Topo 2: leader + 4,5
edges3 = [2 3; 3 4; 4 5; 2 5];      % Topo 3: followers kết nối

T_cycle = 0.3;
phase_durations = [0.1, 0.1, 0.1];

function edges = get_topology(t, T_cycle, pd, e1, e2, e3)
    t_mod = mod(t, T_cycle);
    if t_mod < pd(1)
        edges = e1;
    elseif t_mod < pd(1)+pd(2)
        edges = e2;
    else
        edges = e3;
    end
end

%% ========================================================================
%  3. NHIỄU ĐO LƯỜNG
%% ========================================================================
gps_noise = 0.5;
range_noise = 0.1;
bearing_noise = 0.05;

%% ========================================================================
%  4. KHỞI TẠO BỘ ĐỊNH VỊ
%% ========================================================================
P_est = zeros(dim, n_drones, n_steps);
P_est(:,1,1) = P_true(:,1,1);
for i = 2:n_drones
    P_est(:,i,1) = 30 * randn(dim,1);
end

% Tham số
mu = 5.0;
mu_gps = 2.0;

localization_error = zeros(n_steps, 1);

%% ========================================================================
%  5. VÒNG LẶP ĐỊNH VỊ
%% ========================================================================
fprintf('=== ĐỊNH VỊ ĐỘI HÌNH VỚI SWITCHING TOPOLOGY ===\n');

for k = 1:n_steps-1
    % Cập nhật GPS leader
    P_est(:,1,k+1) = P_true(:,1,k+1);
    
    % Xác định topology
    current_edges = get_topology(t(k), T_cycle, phase_durations, edges1, edges2, edges3);
    
    % Đo vector tương đối
    z_meas = zeros(dim, n_drones, n_drones);
    for e = 1:size(current_edges,1)
        i = current_edges(e,1);
        j = current_edges(e,2);
        
        vec_true = P_true(:,j,k) - P_true(:,i,k);
        d_true = norm(vec_true);
        d_meas = d_true + range_noise * randn;
        dir_meas = vec_true/d_true + bearing_noise * randn(3,1);
        dir_meas = dir_meas / norm(dir_meas);
        z_meas(:,j,i) = d_meas * dir_meas;
        z_meas(:,i,j) = -z_meas(:,j,i);
    end
    
    % Luật cập nhật (8.1): dP_i/dt = Σ (ẑ_ij - z_ij)
    P_next = P_est(:,:,k);
    for i = 1:n_drones
        sum_error = zeros(dim,1);
        for e = 1:size(current_edges,1)
            if current_edges(e,1) == i
                j = current_edges(e,2);
                z_est = P_est(:,j,k) - P_est(:,i,k);
                sum_error = sum_error + (z_est - z_meas(:,j,i));
            elseif current_edges(e,2) == i
                j = current_edges(e,1);
                z_est = P_est(:,j,k) - P_est(:,i,k);
                sum_error = sum_error + (z_est - z_meas(:,j,i));
            end
        end
        dP = mu * sum_error;
        P_next(:,i) = P_est(:,i,k) + dP * dt;
    end
    P_est(:,:,k+1) = P_next;
    P_est(:,1,k+1) = P_true(:,1,k+1) + gps_noise * randn(dim,1);
    
    % Sai số
    loc_error = 0;
    for i = 1:n_drones
        loc_error = loc_error + norm(P_est(:,i,k+1) - P_true(:,i,k+1))^2;
    end
    localization_error(k+1) = sqrt(loc_error / n_drones);
    
    if mod(k, round(n_steps/10)) == 0
        fprintf('t = %.1f s, loc err = %.3f m\n', k*dt, localization_error(k));
    end
end

fprintf('Sai số cuối: %.4f m\n', localization_error(end));

%% ========================================================================
%  6. VẼ KẾT QUẢ
%% ========================================================================
figure('Position', [50, 50, 1200, 500]);

subplot(1,2,1);
colors = lines(n_drones);
hold on; grid on; box on;
for i = 1:n_drones
    plot3(squeeze(P_true(1,i,:)), squeeze(P_true(2,i,:)), squeeze(P_true(3,i,:)), ...
        'Color', colors(i,:), 'LineWidth', 1.5, 'LineStyle', '-');
    plot3(squeeze(P_est(1,i,:)), squeeze(P_est(2,i,:)), squeeze(P_est(3,i,:)), ...
        'Color', colors(i,:), 'LineWidth', 1.5, 'LineStyle', '--');
end
xlabel('x'); ylabel('y'); zlabel('z');
title('Định vị (thật: liền, ước lượng: đứt)');
view(45,30); axis equal;

subplot(1,2,2);
plot(t, localization_error, 'b-', 'LineWidth', 2);
xlabel('Thời gian (s)'); ylabel('Sai số RMS (m)');
title('Sai số định vị');
grid on;

sgtitle('Định vị đội hình với switching topology (công thức 8.1)', 'FontSize', 14);