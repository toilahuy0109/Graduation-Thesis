%% ============================================
%  MPC CHO ĐỘI HÌNH DRONE BÁM THEO CON MỒI (UGV)
%  SỬ DỤNG PARTICLE FILTER ĐỂ DỰ BÁO
%  CÓ PHÂN TÍCH NHIỄU VÀ PHÂN PHỐI
% ============================================

clc; clear; close all;
addpath('./'); % Đảm bảo hàm truncnorm_rvs có sẵn

%% 1. THAM SỐ MÔ PHỎNG CHUNG
% ============================================
dt = 0.05;          % Bước thời gian (s)
T_sim = 30.0;       % Tổng thời gian mô phỏng (s)
N_steps = round(T_sim / dt);
time = linspace(0, T_sim, N_steps);

% Tham số mô hình con mồi (xe UGV)
L = 2.5;            % Chiều dài cơ sở (m)
v_prey = 8.0;       % Vận tốc dài (m/s), không đổi

% Giới hạn góc đánh lái
phi_min_deg = -30;   phi_max_deg = 30;
phi_min = deg2rad(phi_min_deg); phi_max = deg2rad(phi_max_deg);

% Nhiễu góc đánh lái (Gaussian)
mu_phi = 0.0;
sigma_phi_deg = 8;   % Độ lệch chuẩn (độ)
sigma_phi = deg2rad(sigma_phi_deg);
a_norm = (phi_min - mu_phi) / sigma_phi;
b_norm = (phi_max - mu_phi) / sigma_phi;

% Bộ lọc low-pass cho góc lái (mô phỏng quán tính)
fs = 1/dt;
cutoff_freq = 0.5;
[b_lp, a_lp] = butter(2, cutoff_freq/(fs/2), 'low');

%% 2. THAM SỐ ĐỘI HÌNH DRONE
% ============================================
n_drones = 10;              % Số drone trong đội hình (kể cả leader)
R_form = 5.0;               % Bán kính đội hình (m)
% Tạo vị trí tương đối (r0) của các drone trong hệ tọa độ đội hình (tâm tại 0)
r0 = zeros(n_drones, 2);
r0(1,:) = [0, 0];           % Leader ở tâm
angles = linspace(0, 2*pi, n_drones)';
for i = 2:n_drones
    r0(i,:) = R_form * [cos(angles(i)), sin(angles(i))];
end

R_obs = 12.0;               % Bán kính quan sát của drone (m)

%% 3. THAM SỐ MPC & PARTICLE FILTER
% ============================================
N_horizon = 10;             % Số bước dự báo trong tương lai
N_particles = 100;          % Số lượng particle (quỹ đạo mẫu) để ước lượng

% Trọng số hàm mục tiêu
w_move = 1.0;               % Chi phí di chuyển (giữ đội hình ổn định)
w_obs = 500.0;              % Chi phí nếu không đủ drone quan sát
w_angle = 0.5;              % Chi phí thay đổi góc quay

% Ngưỡng quan sát: muốn có ít nhất N_req drone nhìn thấy con mồi
N_req = 5;

% Tham số giới hạn cho bộ tối ưu (bounded optimization)
step_size = 0.5;            % Bước di chuyển tối đa mỗi bước (m, rad)

%% 4. KHỞI TẠO CON MỒI (UGV) VÀ ĐỘI HÌNH DRONE
% ============================================
% --- Khởi tạo con mồi ---
x_prey = zeros(N_steps, 1);
y_prey = zeros(N_steps, 1);
theta_prey = zeros(N_steps, 1);
phi_prey_raw = zeros(N_steps, 1);
phi_prey_smooth = zeros(N_steps, 1);

x_prey(1) = 0; y_prey(1) = 0; theta_prey(1) = 0;

rng(42); % Fix seed
for i = 1:N_steps-1
    phi_raw = truncnorm_rvs(a_norm, b_norm, mu_phi, sigma_phi, 1);
    phi_prey_raw(i) = phi_raw;
end
% Gán giá trị cuối cùng (để đồng bộ kích thước)
phi_prey_raw(N_steps) = phi_prey_raw(N_steps-1);

phi_prey_smooth = filtfilt(b_lp, a_lp, phi_prey_raw);

for i = 1:N_steps-1
    theta_dot = (v_prey * tan(phi_prey_smooth(i))) / L;
    theta_prey(i+1) = theta_prey(i) + theta_dot * dt;
    x_prey(i+1) = x_prey(i) + v_prey * cos(theta_prey(i)) * dt;
    y_prey(i+1) = y_prey(i) + v_prey * sin(theta_prey(i)) * dt;
end

% --- Khởi tạo đội hình drone (bắt đầu tại gốc tọa độ)---
C = [0; 0];               % Tâm đội hình
theta_form = 0;           % Góc quay đội hình
% Lưu lại lịch sử để vẽ
C_hist = zeros(N_steps, 2);
theta_form_hist = zeros(N_steps, 1);
J_hist = zeros(N_steps, 1);

%% 5. VÒNG LẶP MÔ PHỎNG MPC
% ============================================
fprintf('=== BẮT ĐẦU MÔ PHỎNG MPC ===\n');

for k = 1:N_steps
    % Vị trí hiện tại của con mồi
    prey_pos = [x_prey(k); y_prey(k)];
    prey_theta = theta_prey(k);
    
    % --- 5.1. DỰ BÁO TƯƠNG LAI (PARTICLE FILTER) ---
    particles_x = zeros(N_particles, N_horizon+1);
    particles_y = zeros(N_particles, N_horizon+1);
    particles_theta = zeros(N_particles, N_horizon+1);
    
    for p = 1:N_particles
        particles_x(p, 1) = prey_pos(1);
        particles_y(p, 1) = prey_pos(2);
        particles_theta(p, 1) = prey_theta;
        
        for step = 1:N_horizon
            phi_sample = truncnorm_rvs(a_norm, b_norm, mu_phi, sigma_phi, 1);
            if step == 1
                phi_eff = phi_sample;
            else
                phi_eff = 0.7 * phi_eff + 0.3 * phi_sample;
            end
            
            theta_dot_pred = (v_prey * tan(phi_eff)) / L;
            particles_theta(p, step+1) = particles_theta(p, step) + theta_dot_pred * dt;
            particles_x(p, step+1) = particles_x(p, step) + v_prey * cos(particles_theta(p, step)) * dt;
            particles_y(p, step+1) = particles_y(p, step) + v_prey * sin(particles_theta(p, step)) * dt;
        end
    end
    
    % --- 5.2. TỐI ƯU HÓA (GRID SEARCH) ---
    c_x_range = C(1) + linspace(-step_size, step_size, 5);
    c_y_range = C(2) + linspace(-step_size, step_size, 5);
    theta_range = theta_form + linspace(-pi/6, pi/6, 5);
    
    best_J = inf;
    best_C = C;
    best_theta_form = theta_form;
    
    for icx = 1:length(c_x_range)
        for icy = 1:length(c_y_range)
            for itheta = 1:length(theta_range)
                C_test = [c_x_range(icx); c_y_range(icy)];
                theta_test = theta_range(itheta);
                
                total_expected_obs = 0;
                for step = 1:N_horizon
                    expected_N_obs_step = 0;
                    for p = 1:N_particles
                        prey_pos_pred = [particles_x(p, step+1); particles_y(p, step+1)];
                        count_obs = 0;
                        R = [cos(theta_test), -sin(theta_test); sin(theta_test), cos(theta_test)];
                        for i = 1:n_drones
                            drone_pos = C_test + R * r0(i,:)';
                            if norm(drone_pos - prey_pos_pred) <= R_obs
                                count_obs = count_obs + 1;
                            end
                        end
                        expected_N_obs_step = expected_N_obs_step + count_obs;
                    end
                    expected_N_obs_step = expected_N_obs_step / N_particles;
                    total_expected_obs = total_expected_obs + expected_N_obs_step;
                end
                avg_expected_obs = total_expected_obs / N_horizon;
                
                penalty_obs = w_obs * max(0, N_req - avg_expected_obs)^2;
                move_cost = w_move * (norm(C_test - C)^2);
                angle_cost = w_angle * (theta_test - theta_form)^2;
                J_val = penalty_obs + move_cost + angle_cost;
                
                if J_val < best_J
                    best_J = J_val;
                    best_C = C_test;
                    best_theta_form = theta_test;
                end
            end
        end
    end
    
    % --- 5.3. CẬP NHẬT VỊ TRÍ ĐỘI HÌNH ---
    C = best_C;
    theta_form = best_theta_form;
    
    C_hist(k, :) = C';
    theta_form_hist(k) = theta_form;
    J_hist(k) = best_J;
    
    if mod(k, 100) == 0
        fprintf('Thời gian: %.1f s, J = %.2f\n', k*dt, best_J);
    end
end

fprintf('=== MÔ PHỎNG HOÀN TẤT ===\n');

%% ==================== 6. TÍNH TOÁN CÁC ĐẠI LƯỢNG ====================
% Số drone quan sát thực tế theo thời gian
actual_obs_hist = zeros(N_steps, 1);
dist_center_prey = zeros(N_steps, 1);

for k = 1:N_steps
    C_k = C_hist(k,:)';
    theta_k = theta_form_hist(k);
    prey_pos_k = [x_prey(k); y_prey(k)];
    count_obs = 0;
    R = [cos(theta_k), -sin(theta_k); sin(theta_k), cos(theta_k)];
    for i = 1:n_drones
        drone_pos = C_k + R * r0(i,:)';
        if norm(drone_pos - prey_pos_k) <= R_obs
            count_obs = count_obs + 1;
        end
    end
    actual_obs_hist(k) = count_obs;
    dist_center_prey(k) = norm(C_k - prey_pos_k);
end

% Tính tỷ lệ thời gian đạt ngưỡng
compliance_ratio = sum(actual_obs_hist >= N_req) / N_steps * 100;

% Lấy chỉ số bắt đầu (bỏ qua 2s đầu)
start_idx = max(1, round(2/dt));

%% ==================== 7. HÌNH 1: KẾT QUẢ MPC ====================
figure('Position', [50, 50, 1400, 900]);

% (1) Quỹ đạo con mồi và tâm đội hình
subplot(2,4,1);
plot(x_prey, y_prey, 'r-', 'LineWidth', 1.5); hold on;
plot(C_hist(:,1), C_hist(:,2), 'b--', 'LineWidth', 1.5);
plot(x_prey(1), y_prey(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(C_hist(1,1), C_hist(1,2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
plot(x_prey(end), y_prey(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot(C_hist(end,1), C_hist(end,2), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
xlabel('x (m)'); ylabel('y (m)');
title('Quỹ đạo: Con mồi (đỏ) và Tâm đội hình (xanh)');
legend('Con mồi', 'Tâm đội hình', 'Start', 'End', 'Location', 'best');
grid on; axis equal;

% (2) Số drone quan sát thực tế
subplot(2,4,2);
stairs(time, actual_obs_hist, 'g-', 'LineWidth', 1.5); hold on;
yline(N_req, 'r--', 'LineWidth', 1.5);
xlabel('Thời gian (s)'); ylabel('Số drone quan sát');
title(sprintf('Số drone quan sát (%.1f%% thời gian đạt yêu cầu)', compliance_ratio));
ylim([0, n_drones+2]);
grid on;
legend('Thực tế', sprintf('Ngưỡng %d', N_req), 'Location', 'best');

% (3) Giá trị hàm mục tiêu J
subplot(2,4,3);
semilogy(time(start_idx:end), J_hist(start_idx:end), 'm-', 'LineWidth', 1.5);
xlabel('Thời gian (s)'); ylabel('J (log scale)');
title('Hàm mục tiêu (chi phí)');
grid on;

% (4) Khoảng cách tâm đội hình - con mồi
subplot(2,4,4);
plot(time, dist_center_prey, 'b-', 'LineWidth', 1.5);
xlabel('Thời gian (s)'); ylabel('Khoảng cách (m)');
title('Khoảng cách tâm đội hình → con mồi');
grid on;

% (5) Góc quay đội hình
subplot(2,4,5);
plot(time, rad2deg(theta_form_hist), 'c-', 'LineWidth', 1.5);
xlabel('Thời gian (s)'); ylabel('Góc quay (độ)');
title('Góc quay đội hình');
grid on;

% (6) Histogram góc đánh lái
subplot(2,4,6);
phi_deg = rad2deg(phi_prey_smooth);
histogram(phi_deg, 30, 'Normalization', 'pdf', 'FaceColor', [0.2, 0.5, 0.8]);
hold on;
xline(phi_min_deg, 'r--', 'LineWidth', 1.5);
xline(phi_max_deg, 'r--', 'LineWidth', 1.5);
phi_theory = linspace(phi_min_deg, phi_max_deg, 200);
pdf_theory = normpdf(phi_theory, rad2deg(mu_phi), rad2deg(sigma_phi));
% Chuẩn hóa PDF
pdf_theory = pdf_theory / (normcdf(phi_max_deg, rad2deg(mu_phi), rad2deg(sigma_phi)) - normcdf(phi_min_deg, rad2deg(mu_phi), rad2deg(sigma_phi)));
plot(phi_theory, pdf_theory, 'k-', 'LineWidth', 2);
xlabel('Góc đánh lái φ (độ)'); ylabel('Mật độ xác suất');
title('Phân phối góc đánh lái');
legend('Thực tế', 'Giới hạn', 'Gaussian truncated', 'Location', 'best');
grid on;

% (7) Biến thiên góc lái theo thời gian
subplot(2,4,7);
plot(time, rad2deg(phi_prey_smooth), 'r-', 'LineWidth', 1.2);
hold on;
yline(phi_min_deg, 'k--', 'LineWidth', 1);
yline(phi_max_deg, 'k--', 'LineWidth', 1);
xlabel('Thời gian (s)'); ylabel('Góc đánh lái φ (độ)');
title('Biến thiên góc lái (sau low-pass)');
legend('Smoothed', 'Giới hạn', 'Location', 'best');
grid on;

% (8) Snapshot đội hình cuối cùng
subplot(2,4,8);
R_final = [cos(theta_form_hist(end)), -sin(theta_form_hist(end)); sin(theta_form_hist(end)), cos(theta_form_hist(end))];
final_drones = zeros(n_drones, 2);
for i = 1:n_drones
    final_drones(i,:) = (C_hist(end,:)' + R_final * r0(i,:)')';
end
plot(final_drones(:,1), final_drones(:,2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b'); hold on;
plot(C_hist(end,1), C_hist(end,2), 'k^', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
plot(x_prey(end), y_prey(end), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
theta_circ = linspace(0, 2*pi, 100);
plot(x_prey(end) + R_obs*cos(theta_circ), y_prey(end) + R_obs*sin(theta_circ), 'r--', 'LineWidth', 1);
xlabel('x (m)'); ylabel('y (m)');
title(sprintf('Trạng thái cuối: N_{obs} = %.0f', actual_obs_hist(end)));
axis equal; grid on;
legend('Drone', 'Tâm', 'Con mồi', 'Vùng quan sát', 'Location', 'best');

sgtitle('HÌNH 1: KẾT QUẢ MPC - ĐIỀU KHIỂN ĐỘI HÌNH BÁM THEO CON MỒI');

%% ==================== 8. HÌNH 2: PHÂN TÍCH NHIỄU CHI TIẾT ====================
figure('Position', [100, 100, 1200, 500]);

% (1) So sánh raw và smoothed (chỉ hiển thị 5s đầu cho dễ nhìn)
subplot(1,3,1);
t_short = 5; % chỉ hiển thị 5s đầu
idx_short = time <= t_short;
plot(time(idx_short), rad2deg(phi_prey_raw(idx_short)), 'b-', 'LineWidth', 0.5);
hold on;
plot(time(idx_short), rad2deg(phi_prey_smooth(idx_short)), 'r-', 'LineWidth', 1.5);
yline(phi_min_deg, 'k--', 'LineWidth', 1);
yline(phi_max_deg, 'k--', 'LineWidth', 1);
xlabel('Thời gian (s)'); ylabel('Góc đánh lái (độ)');
title('Nhiễu góc lái (5s đầu)');
legend('Raw Gaussian noise', 'After low-pass filter', 'Mechanical limits', 'Location', 'best');
grid on;
xlim([0, t_short]);

% (2) Q-Q plot kiểm tra tính Gaussian
subplot(1,3,2);
phi_normalized = (phi_prey_smooth - mu_phi) / sigma_phi;
qqplot(phi_normalized);
title('Q-Q Plot: So sánh với phân phối chuẩn');
xlabel('Phân vị lý thuyết'); ylabel('Phân vị dữ liệu');
grid on;

% (3) Autocorrelation của nhiễu
subplot(1,3,3);
[acf, lags] = xcorr(phi_prey_smooth - mean(phi_prey_smooth), 50, 'normalized');
lags = lags(51:end);
acf = acf(51:end);
stem(lags, acf, 'filled', 'MarkerSize', 3);
xlabel('Lag'); ylabel('Autocorrelation');
title('Autocorrelation của nhiễu góc lái');
yline(1.96/sqrt(N_steps), 'r--', 'LineWidth', 1);
yline(-1.96/sqrt(N_steps), 'r--', 'LineWidth', 1);
grid on;
legend('ACF', '95% confidence bound');

sgtitle('HÌNH 2: PHÂN TÍCH THỐNG KÊ NHIỄU CON MỒI');

%% ==================== 9. IN BÁO CÁO THỐNG KÊ ====================
fprintf('\n==================== BÁO CÁO THỐNG KÊ ====================\n');
fprintf('--- NHIỄU GÓC LÁI ---\n');
fprintf('  Kỳ vọng thiết kế: μ = %.2f°\n', rad2deg(mu_phi));
fprintf('  Độ lệch chuẩn thiết kế: σ = %.2f°\n', rad2deg(sigma_phi));
fprintf('  Kỳ vọng thực tế: μ = %.3f°\n', rad2deg(mean(phi_prey_smooth)));
fprintf('  Độ lệch chuẩn thực tế: σ = %.3f°\n', rad2deg(std(phi_prey_smooth)));
fprintf('  Giới hạn cơ khí: [%.1f°, %.1f°]\n', phi_min_deg, phi_max_deg);
fprintf('  Min/Max thực tế: [%.2f°, %.2f°]\n', rad2deg(min(phi_prey_smooth)), rad2deg(max(phi_prey_smooth)));
fprintf('  Tỷ lệ ngoài giới hạn: %.4f%%\n', sum(abs(phi_prey_smooth) > phi_max) / length(phi_prey_smooth) * 100);

fprintf('\n--- QUAN SÁT ---\n');
fprintf('  Yêu cầu: N_req = %d drone\n', N_req);
fprintf('  Tỷ lệ thời gian đạt yêu cầu: %.2f%%\n', compliance_ratio);
fprintf('  Số drone quan sát trung bình: %.2f\n', mean(actual_obs_hist));
fprintf('  Số drone quan sát min/max: %.0f/%.0f\n', min(actual_obs_hist), max(actual_obs_hist));

fprintf('\n--- BÁM THEO ---\n');
fprintf('  Khoảng cách TB tâm-mồi: %.2f m\n', mean(dist_center_prey));
fprintf('  Khoảng cách cuối cùng: %.2f m\n', dist_center_prey(end));
fprintf('  Khoảng cách min/max: %.2f/%.2f m\n', min(dist_center_prey), max(dist_center_prey));

fprintf('\n--- HÀM MỤC TIÊU ---\n');
fprintf('  J trung bình (sau 2s): %.2f\n', mean(J_hist(start_idx:end)));
fprintf('  J cuối cùng: %.2f\n', J_hist(end));
fprintf('=============================================================\n');

%% ==================== 10. HÀM HỖ TRỢ ====================
function rvs = truncnorm_rvs(a, b, mu, sigma, n)
    % a, b: normalized bounds: (lower - mu)/sigma, (upper - mu)/sigma
    % mu, sigma: parameters of original Gaussian
    % n: number of samples
    
    Phi_a = normcdf(a);
    Phi_b = normcdf(b);
    u = rand(n, 1) * (Phi_b - Phi_a) + Phi_a;
    z = norminv(u);
    rvs = mu + sigma * z;
end