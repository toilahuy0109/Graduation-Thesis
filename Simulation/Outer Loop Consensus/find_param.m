%% ============================================
%  MONTE CARLO SIMULATION - ĐÁNH GIÁ HỘI TỤ
%  MPC CHO ĐỘI HÌNH DRONE BÁM THEO CON MỒI (UGV)
% ============================================
%  Mục đích: Chạy N_sim lần mô phỏng với các hạt nhiễu khác nhau
%            để đánh giá tính ổn định và khả năng hội tụ của thuật toán.
% ============================================

clc; clear; close all;

%% ==================== 1. THAM SỐ MONTE CARLO ====================
N_sim = 50;                 % Số lần mô phỏng (kịch bản)
use_parallel = false;       % Bật parallel computing nếu có (chạy nhanh hơn)

% Các tham số cần lưu cho mỗi kịch bản
results = struct();
results.N_sim = N_sim;

%% ==================== 2. THAM SỐ MÔ PHỎNG CHUNG ====================
dt = 0.05;                  % Bước thời gian (s)
T_sim = 30.0;               % Tổng thời gian mô phỏng (s)
N_steps = round(T_sim / dt);
time = linspace(0, T_sim, N_steps)';

% Tham số mô hình con mồi (xe UGV)
L = 2.5;                    % Chiều dài cơ sở (m)
v_prey = 8.0;               % Vận tốc dài (m/s), không đổi

% Giới hạn góc đánh lái
phi_min_deg = -30;   phi_max_deg = 30;
phi_min = deg2rad(phi_min_deg); phi_max = deg2rad(phi_max_deg);

% Nhiễu góc đánh lái (Gaussian)
mu_phi = 0.0;
sigma_phi_deg = 8;          % Độ lệch chuẩn (độ)
sigma_phi = deg2rad(sigma_phi_deg);
a_norm = (phi_min - mu_phi) / sigma_phi;
b_norm = (phi_max - mu_phi) / sigma_phi;

% Bộ lọc low-pass cho góc lái
fs = 1/dt;
cutoff_freq = 0.5;
[b_lp, a_lp] = butter(2, cutoff_freq/(fs/2), 'low');

%% ==================== 3. THAM SỐ ĐỘI HÌNH DRONE ====================
n_drones = 10;
R_form = 5.0;
r0 = zeros(n_drones, 2);
r0(1,:) = [0, 0];
angles = linspace(0, 2*pi, n_drones)';
for i = 2:n_drones
    r0(i,:) = R_form * [cos(angles(i)), sin(angles(i))];
end
R_obs = 12.0;

%% ==================== 4. THAM SỐ MPC & PARTICLE FILTER ====================
N_horizon = 10;
N_particles = 100;          % Số particle dự báo

% Trọng số hàm mục tiêu
w_move = 1.0;
w_obs = 500.0;
w_angle = 0.5;
N_req = 5;                  % Ngưỡng số drone quan sát

% Tham số tối ưu
step_size = 0.5;            % Bước tìm kiếm tâm (m)
grid_points = 5;            % Số điểm lưới mỗi chiều

% Hệ số cập nhật (smoothing)
alpha_smooth = 0.5;

%% ==================== 5. LƯU TRỮ KẾT QUẢ ====================
% Lưu tất cả các kịch bản
all_prey_x = zeros(N_sim, N_steps);
all_prey_y = zeros(N_sim, N_steps);
all_phi_smooth = zeros(N_sim, N_steps);
all_C_x = zeros(N_sim, N_steps);
all_C_y = zeros(N_sim, N_steps);
all_theta_form = zeros(N_sim, N_steps);
all_N_obs = zeros(N_sim, N_steps);
all_J = zeros(N_sim, N_steps);
all_dist_center_prey = zeros(N_sim, N_steps);

%% ==================== 6. VÒNG LẶP MONTE CARLO ====================
fprintf('=== BẮT ĐẦU MONTE CARLO VỚI %d KỊCH BẢN ===\n', N_sim);
tic;

% Chọn phương thức chạy
if use_parallel && license('test', 'Distrib_Computing_Toolbox')
    % Parallel Computing (nếu có)
    parfor sim = 1:N_sim
        [results_sim] = run_single_simulation(sim, dt, T_sim, N_steps, ...
            L, v_prey, phi_min, phi_max, mu_phi, sigma_phi, a_norm, b_norm, ...
            b_lp, a_lp, n_drones, r0, R_obs, R_form, ...
            N_horizon, N_particles, w_move, w_obs, w_angle, N_req, ...
            step_size, grid_points, alpha_smooth);
        
        % Lưu kết quả (dùng cell array vì parfor)
        prey_x_sim{sim} = results_sim.prey_x;
        prey_y_sim{sim} = results_sim.prey_y;
        phi_smooth_sim{sim} = results_sim.phi_smooth;
        C_x_sim{sim} = results_sim.C_x;
        C_y_sim{sim} = results_sim.C_y;
        theta_form_sim{sim} = results_sim.theta_form;
        N_obs_sim{sim} = results_sim.N_obs;
        J_sim{sim} = results_sim.J;
        dist_sim{sim} = results_sim.dist_center_prey;
    end
    
    % Chuyển từ cell sang array
    for sim = 1:N_sim
        all_prey_x(sim, :) = prey_x_sim{sim};
        all_prey_y(sim, :) = prey_y_sim{sim};
        all_phi_smooth(sim, :) = phi_smooth_sim{sim};
        all_C_x(sim, :) = C_x_sim{sim};
        all_C_y(sim, :) = C_y_sim{sim};
        all_theta_form(sim, :) = theta_form_sim{sim};
        all_N_obs(sim, :) = N_obs_sim{sim};
        all_J(sim, :) = J_sim{sim};
        all_dist_center_prey(sim, :) = dist_sim{sim};
    end
    
else
    % Sequential (tuần tự) - đơn giản, dễ debug
    for sim = 1:N_sim
        fprintf('  Kịch bản %d/%d...\n', sim, N_sim);
        
        [results_sim] = run_single_simulation(sim, dt, T_sim, N_steps, ...
            L, v_prey, phi_min, phi_max, mu_phi, sigma_phi, a_norm, b_norm, ...
            b_lp, a_lp, n_drones, r0, R_obs, R_form, ...
            N_horizon, N_particles, w_move, w_obs, w_angle, N_req, ...
            step_size, grid_points, alpha_smooth);
        
        all_prey_x(sim, :) = results_sim.prey_x;
        all_prey_y(sim, :) = results_sim.prey_y;
        all_phi_smooth(sim, :) = results_sim.phi_smooth;
        all_C_x(sim, :) = results_sim.C_x;
        all_C_y(sim, :) = results_sim.C_y;
        all_theta_form(sim, :) = results_sim.theta_form;
        all_N_obs(sim, :) = results_sim.N_obs;
        all_J(sim, :) = results_sim.J;
        all_dist_center_prey(sim, :) = results_sim.dist_center_prey;
    end
end

elapsed_time = toc;
fprintf('=== HOÀN THÀNH %d KỊCH BẢN TRONG %.2f GIÂY ===\n', N_sim, elapsed_time);

%% ==================== 7. TÍNH TOÁN THỐNG KÊ ====================
% Tính trung bình và độ lệch chuẩn
mean_C_x = mean(all_C_x, 1);
std_C_x = std(all_C_x, 0, 1);
mean_C_y = mean(all_C_y, 1);
std_C_y = std(all_C_y, 0, 1);

mean_N_obs = mean(all_N_obs, 1);
std_N_obs = std(all_N_obs, 0, 1);

mean_J = mean(all_J, 1);
std_J = std(all_J, 0, 1);

mean_dist = mean(all_dist_center_prey, 1);
std_dist = std(all_dist_center_prey, 0, 1);

mean_theta_form = mean(all_theta_form, 1);
std_theta_form = std(all_theta_form, 0, 1);

% Tính tỷ lệ thời gian đạt N_req
N_obs_meet_req = all_N_obs >= N_req;
fraction_meet_req = mean(N_obs_meet_req, 1);
overall_compliance = mean(fraction_meet_req) * 100;

% Tính thống kê nhiễu góc lái
all_phi_flat = all_phi_smooth(:);
phi_mean_deg = rad2deg(mean(all_phi_flat));
phi_std_deg = rad2deg(std(all_phi_flat));
phi_min_actual_deg = rad2deg(min(all_phi_flat));
phi_max_actual_deg = rad2deg(max(all_phi_flat));

%% ==================== 8. VẼ BIỂU ĐỒ PHÂN TÍCH NHIỄU ====================
figure('Position', [50, 50, 1600, 900]);

% 8.1. Histogram góc đánh lái
subplot(2,3,1);
histogram(rad2deg(all_phi_flat), 50, 'Normalization', 'pdf', 'FaceColor', [0.2, 0.6, 0.8]);
hold on;
xline(phi_min_deg, 'r--', 'LineWidth', 1.5);
xline(phi_max_deg, 'r--', 'LineWidth', 1.5);
xlabel('Góc đánh lái \phi (độ)');
ylabel('Mật độ xác suất');
title(sprintf('Phân phối góc lái (N = %d)', N_sim));
legend('PDF', sprintf('Giới hạn [%d°, %d°]', phi_min_deg, phi_max_deg), 'Location', 'best');
grid on;

% 8.2. Chuỗi thời gian góc lái (mẫu)
subplot(2,3,2);
plot(time, rad2deg(all_phi_smooth(1:min(5,N_sim), :))', 'LineWidth', 0.8);
hold on;
yline(phi_min_deg, 'r--', 'LineWidth', 1);
yline(phi_max_deg, 'r--', 'LineWidth', 1);
xlabel('Thời gian (s)');
ylabel('Góc đánh lái \phi (độ)');
title('Biến thiên góc lái theo thời gian (5 mẫu đầu)');
grid on;

% 8.3. Quỹ đạo con mồi (tất cả kịch bản)
subplot(2,3,3);
for sim = 1:min(N_sim, 20)
    plot(all_prey_x(sim, :), all_prey_y(sim, :), 'Color', [0.7, 0.3, 0.3, 0.3], 'LineWidth', 0.5);
    hold on;
end
plot(mean(all_prey_x, 1), mean(all_prey_y, 1), 'r-', 'LineWidth', 2);
xlabel('x (m)'); ylabel('y (m)');
title('Quỹ đạo con mồi (xám: các kịch bản, đỏ: trung bình)');
axis equal; grid on;

%% ==================== 9. VẼ BIỂU ĐỒ HỘI TỤ ====================

% 9.1. Số drone quan sát (trung bình ± std)
subplot(2,3,4);
hold on;
fill([time, fliplr(time)], [mean_N_obs+std_N_obs, fliplr(mean_N_obs-std_N_obs)], ...
     [0.8, 0.9, 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
plot(time, mean_N_obs, 'g-', 'LineWidth', 2);
yline(N_req, 'r--', 'LineWidth', 1.5);
xlabel('Thời gian (s)'); ylabel('Số drone quan sát');
title(sprintf('Số drone quan sát (trung bình ± 1σ)\nĐạt yêu cầu: %.1f%% thời gian', overall_compliance));
ylim([0, n_drones+2]);
grid on;
legend('±1σ', 'Trung bình', sprintf('Ngưỡng %d', N_req), 'Location', 'best');

% 9.2. Khoảng cách tâm đội hình - con mồi
subplot(2,3,5);
hold on;
fill([time, fliplr(time)], [mean_dist+std_dist, fliplr(mean_dist-std_dist)], ...
     [0.8, 0.8, 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
plot(time, mean_dist, 'b-', 'LineWidth', 2);
xlabel('Thời gian (s)'); ylabel('Khoảng cách (m)');
title('Khoảng cách từ tâm đội hình đến con mồi (trung bình ± 1σ)');
grid on;

% 9.3. Giá trị hàm mục tiêu J
subplot(2,3,6);
hold on;
% Lấy từ bước 10 trở đi để bỏ qua transient
start_idx = max(1, round(2/dt));
semilogy(time(start_idx:end), mean_J(start_idx:end), 'm-', 'LineWidth', 2);
xlabel('Thời gian (s)'); ylabel('J (log scale)');
title('Hàm mục tiêu J (trung bình, bỏ qua 2s đầu)');
grid on;

sgtitle(sprintf('KẾT QUẢ MONTE CARLO: %d kịch bản, T_{sim} = %.1f s', N_sim, T_sim));

%% ==================== 10. BIỂU ĐỒ SO SÁNH THÊM ====================
figure('Position', [100, 100, 1200, 800]);

% 10.1. Histogram phân phối khoảng cách cuối cùng
subplot(2,2,1);
final_dist = all_dist_center_prey(:, end);
histogram(final_dist, 30, 'Normalization', 'pdf', 'FaceColor', [0.3, 0.5, 0.7]);
xlabel('Khoảng cách cuối (m)'); ylabel('Mật độ');
title('Phân phối khoảng cách tâm-mồi tại thời điểm cuối');
grid on;

% 10.2. Histogram tỷ lệ đạt ngưỡng
subplot(2,2,2);
compliance_per_sim = mean(N_obs_meet_req, 2) * 100;
histogram(compliance_per_sim, 20, 'FaceColor', [0.2, 0.7, 0.3]);
xlabel('Tỷ lệ thời gian đạt N_{obs} ≥ N_{req} (%)');
ylabel('Số kịch bản');
title(sprintf('Phân bố tỷ lệ tuân thủ (trung bình = %.1f%%)', overall_compliance));
xlim([0, 100]);
grid on;

% 10.3. Quan hệ giữa khoảng cách trung bình và độ lệch chuẩn
subplot(2,2,3);
mean_dist_per_sim = mean(all_dist_center_prey, 2);
std_dist_per_sim = std(all_dist_center_prey, 0, 2);
scatter(mean_dist_per_sim, std_dist_per_sim, 40, 'filled', 'MarkerFaceColor', [0.5, 0.3, 0.8]);
xlabel('Khoảng cách trung bình (m)'); ylabel('Độ lệch chuẩn (m)');
title('Tương quan giữa độ chính xác và độ ổn định');
grid on;

% 10.4. Đường trung bình góc quay đội hình
subplot(2,2,4);
plot(time, rad2deg(mean_theta_form), 'c-', 'LineWidth', 2);
hold on;
fill([time, fliplr(time)], rad2deg([mean_theta_form+std_theta_form, fliplr(mean_theta_form-std_theta_form)]), ...
     [0.7, 0.8, 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
xlabel('Thời gian (s)'); ylabel('Góc quay (độ)');
title('Góc quay đội hình (trung bình ± 1σ)');
grid on;

sgtitle(sprintf('PHÂN TÍCH HỘI TỤ & ỔN ĐỊNH - %d KỊCH BẢN', N_sim));

%% ==================== 11. BÁO CÁO THỐNG KÊ ====================
fprintf('\n==================== BÁO CÁO THỐNG KÊ ====================\n');
fprintf('Số kịch bản: %d\n', N_sim);
fprintf('Thời gian mô phỏng mỗi kịch bản: %.1f s\n', T_sim);
fprintf('\n--- NHIỄU GÓC LÁI ---\n');
fprintf('  Kỳ vọng thiết kế: μ = 0°, σ = %.1f°\n', sigma_phi_deg);
fprintf('  Kỳ vọng thực tế: μ = %.2f°, σ = %.2f°\n', phi_mean_deg, phi_std_deg);
fprintf('  Min/Max thực tế: [%.1f°, %.1f°]\n', phi_min_actual_deg, phi_max_actual_deg);
fprintf('\n--- QUAN SÁT ---\n');
fprintf('  Yêu cầu: N_req = %d drone\n', N_req);
fprintf('  Tỷ lệ tuân thủ trung bình: %.1f%%\n', overall_compliance);
fprintf('  Số drone quan sát trung bình: %.2f ± %.2f\n', mean(mean_N_obs), mean(std_N_obs));
fprintf('\n--- BÁM THEO ---\n');
fprintf('  Khoảng cách TB cuối: %.2f ± %.2f m\n', mean(final_dist), std(final_dist));
fprintf('  Khoảng cách TB toàn thời gian: %.2f ± %.2f m\n', mean(mean_dist_per_sim), mean(std_dist_per_sim));
fprintf('  Hàm mục tiêu J TB (sau 2s): %.2f\n', mean(mean_J(round(2/dt):end)));
fprintf('=============================================================\n');

%% ==================== 12. HÀM CHẠY MỘT KỊCH BẢN ====================
function results = run_single_simulation(sim_id, dt, T_sim, N_steps, ...
    L, v_prey, phi_min, phi_max, mu_phi, sigma_phi, a_norm, b_norm, ...
    b_lp, a_lp, n_drones, r0, R_obs, R_form, ...
    N_horizon, N_particles, w_move, w_obs, w_angle, N_req, ...
    step_size, grid_points, alpha_smooth)

    % Set seed riêng cho mỗi kịch bản
    rng(sim_id * 12345);
    
    time = linspace(0, T_sim, N_steps)';
    
    % --- KHỞI TẠO CON MỒI ---
    x_prey = zeros(N_steps, 1);
    y_prey = zeros(N_steps, 1);
    theta_prey = zeros(N_steps, 1);
    phi_prey_raw = zeros(N_steps, 1);
    
    x_prey(1) = 0; y_prey(1) = 0; theta_prey(1) = 0;
    
    for i = 1:N_steps-1
        phi_raw = truncnorm_rvs(a_norm, b_norm, mu_phi, sigma_phi, 1);
        phi_prey_raw(i) = phi_raw;
    end
    phi_prey_smooth = filtfilt(b_lp, a_lp, phi_prey_raw);
    
    for i = 1:N_steps-1
        theta_dot = (v_prey * tan(phi_prey_smooth(i))) / L;
        theta_prey(i+1) = theta_prey(i) + theta_dot * dt;
        x_prey(i+1) = x_prey(i) + v_prey * cos(theta_prey(i)) * dt;
        y_prey(i+1) = y_prey(i) + v_prey * sin(theta_prey(i)) * dt;
    end
    
    % --- KHỞI TẠO ĐỘI HÌNH ---
    C = [0; 0];
    theta_form = 0;
    
    C_x_hist = zeros(N_steps, 1);
    C_y_hist = zeros(N_steps, 1);
    theta_form_hist = zeros(N_steps, 1);
    N_obs_hist = zeros(N_steps, 1);
    J_hist = zeros(N_steps, 1);
    dist_hist = zeros(N_steps, 1);
    
    % --- VÒNG LẶP CHÍNH ---
    for k = 1:N_steps
        prey_pos = [x_prey(k); y_prey(k)];
        prey_theta = theta_prey(k);
        
        % Particle filter dự báo
        particles_x = zeros(N_particles, N_horizon+1);
        particles_y = zeros(N_particles, N_horizon+1);
        particles_theta = zeros(N_particles, N_horizon+1);
        
        for p = 1:N_particles
            particles_x(p, 1) = prey_pos(1);
            particles_y(p, 1) = prey_pos(2);
            particles_theta(p, 1) = prey_theta;
            
            phi_eff = 0;
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
        
        % Tối ưu hóa (grid search)
        c_x_range = C(1) + linspace(-step_size, step_size, grid_points);
        c_y_range = C(2) + linspace(-step_size, step_size, grid_points);
        theta_range = theta_form + linspace(-pi/6, pi/6, grid_points);
        
        best_J = inf;
        best_C = C;
        best_theta_form = theta_form;
        
        for icx = 1:length(c_x_range)
            for icy = 1:length(c_y_range)
                for ith = 1:length(theta_range)
                    C_test = [c_x_range(icx); c_y_range(icy)];
                    theta_test = theta_range(ith);
                    
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
                    move_cost = w_move * norm(C_test - C)^2;
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
        
        % Cập nhật
        C = C + alpha_smooth * (best_C - C);
        theta_form = theta_form + alpha_smooth * (best_theta_form - theta_form);
        
        % Lưu lịch sử
        C_x_hist(k) = C(1);
        C_y_hist(k) = C(2);
        theta_form_hist(k) = theta_form;
        J_hist(k) = best_J;
        
        % Đếm số drone quan sát thực tế
        Rmat = [cos(theta_form), -sin(theta_form); sin(theta_form), cos(theta_form)];
        count_obs = 0;
        for i = 1:n_drones
            drone_pos = C + Rmat * r0(i,:)';
            if norm(drone_pos - prey_pos) <= R_obs
                count_obs = count_obs + 1;
            end
        end
        N_obs_hist(k) = count_obs;
        dist_hist(k) = norm(C - prey_pos);
    end
    
    % Gán kết quả
    results.prey_x = x_prey';
    results.prey_y = y_prey';
    results.phi_smooth = phi_prey_smooth';
    results.C_x = C_x_hist';
    results.C_y = C_y_hist';
    results.theta_form = theta_form_hist';
    results.N_obs = N_obs_hist';
    results.J = J_hist';
    results.dist_center_prey = dist_hist';
end

%% ==================== HÀM TRUNCATED GAUSSIAN ====================
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