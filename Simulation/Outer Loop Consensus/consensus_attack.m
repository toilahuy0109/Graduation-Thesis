%% MAIN SIMULATION - So sánh tốc độ hội tụ khi có tấn công vào dữ liệu giao tiếp
clear; clc; close all;

%% ========================================================================
%  THAM SỐ MÔ PHỎNG
% =========================================================================
dt = 0.02;
T_total = 30;
t = 0:dt:T_total;
N_steps = length(t);

n_drones = 10;
dim = 3;

%% ========================================================================
%  ĐỊNH NGHĨA EDGES (ĐỒ THỊ ĐỦ CỨNG - 24 CẠNH)
% =========================================================================
edges_center = [];
for i = 2:n_drones
    edges_center = [edges_center; 1, i];
end

edges_circle = [];
for i = 2:n_drones-1
    edges_circle = [edges_circle; i, i+1];
end
edges_circle = [edges_circle; n_drones, 2];

edges_diagonal = [
    2, 4;  3, 5;  4, 6;  5, 7;  6, 8;  7, 9;  8, 10;  9, 2;
    2, 6;  3, 7;  4, 8;  5, 9;  6, 10
];

edges = [edges_center; edges_circle; edges_diagonal];
n_edges = size(edges, 1);

%% ========================================================================
%  THAM SỐ ĐIỀU KHIỂN
% =========================================================================
alpha_dist = 0.12;
beta_dist = 40.0;
kp = 0.08;
kd = 0.15;
k_pos = 0.5;
d_safe = 1.0;
d_star = 2.5 * ones(n_edges, 1);
noise_std = 0.03;

%% ========================================================================
%  CẤU HÌNH TẤN CÔNG
% =========================================================================
attack_scenario = 1;  % 0: không tấn công, 1: tấn công mạng

attack_config = struct();
attack_config.start_time = 8;
attack_config.end_time = 20;
attack_config.target_drones = [3, 5, 7];
attack_config.position_bias = [5; 5; 2];
attack_config.position_scale = 1.5;
attack_config.noise_std = 1.0;

%% ========================================================================
%  KHỞI TẠO VỊ TRÍ
% =========================================================================
pos = zeros(3, n_drones);
vel = zeros(3, n_drones);

% Drone trung tâm (drone 4) ở gốc
pos(:, 4) = [0; 0; 0];

% Các drone khác trên vòng tròn
radius = 3.0;
angles = linspace(0, 2*pi, n_drones-1);
idx = 1;
for i = 1:n_drones
    if i == 4; continue; end
    pos(1, i) = radius * cos(angles(idx));
    pos(2, i) = radius * sin(angles(idx));
    pos(3, i) = 0.3 * sin(angles(idx) * 2);
    idx = idx + 1;
end
vel(3, 4) = 0.3;

% Vị trí đặt (đích đến)
pes = zeros(3, n_drones);
pes(:, 1) = [5; 5; 5];
pes(:, 2) = [5; -5; 5];
pes(:, 3) = [-5; 5; 5];
pes(:, 4) = [0; 0; 10];
pes(:, 5) = [-5; -5; 5];
pes(:, 6) = [5; 0; 5];
pes(:, 7) = [-5; 0; 5];
pes(:, 8) = [0; 5; 5];
pes(:, 9) = [0; -5; 5];
pes(:, 10) = [0; 0; 5];

%% ========================================================================
%  KHỞI TẠO BỘ ƯỚC LƯỢNG VỊ TRÍ
% =========================================================================
pos_estimated = pos;

%% ========================================================================
%  MẢNG LƯU TRỮ
% =========================================================================
% Khởi tạo mảng với kích thước rõ ràng
N_save = N_steps;
pos_history = zeros(3, n_drones, N_save);
vel_history = zeros(3, n_drones, N_save);
estimation_error = zeros(N_save, n_drones);
consensus_error = zeros(N_save, 1);

% Lưu trạng thái ban đầu
pos_history(:, :, 1) = pos;
vel_history(:, :, 1) = vel;

%% ========================================================================
%  VÒNG LẶP MÔ PHỎNG
% =========================================================================
fprintf('=== BẮT ĐẦU MÔ PHỎNG ===\n');
if attack_scenario == 1
    fprintf('Kịch bản: Có tấn công mạng\n');
else
    fprintf('Kịch bản: Không có tấn công\n');
end
fprintf('Thời gian tấn công: %.1fs -> %.1fs\n', attack_config.start_time, attack_config.end_time);
fprintf('Drone bị tấn công: ');
fprintf('%d ', attack_config.target_drones);
fprintf('\n');

h = waitbar(0, 'Đang mô phỏng...');

for step = 1:N_steps-1
    if mod(step, round(N_steps/100)) == 0
        waitbar(step/N_steps, h);
    end
    
    current_time = t(step);
    
    % =====================================================================
    %  BƯỚC 1: MÔ PHỎNG TẤN CÔNG VÀO DỮ LIỆU GIAO TIẾP
    % =====================================================================
    % Vị trí ước lượng lý tưởng (có nhiễu nhỏ)
    pos_estimated_ideal = pos + noise_std * randn(3, n_drones);
    
    % Khởi tạo vị trí ước lượng
    pos_estimated = pos_estimated_ideal;
    
    % Tấn công nếu đang trong thời gian tấn công
    if attack_scenario == 1 && current_time >= attack_config.start_time && current_time <= attack_config.end_time
        for i = 1:n_drones
            if ismember(i, attack_config.target_drones)
                % Drone bị tấn công gửi vị trí sai
                pos_estimated(:, i) = pos_estimated_ideal(:, i) + attack_config.position_bias;
                pos_estimated(:, i) = pos_estimated(:, i) * attack_config.position_scale;
                pos_estimated(:, i) = pos_estimated(:, i) + attack_config.noise_std * randn(3,1);
            end
        end
    end
    
    % Lưu lỗi ước lượng
    for i = 1:n_drones
        estimation_error(step, i) = norm(pos_estimated(:, i) - pos(:, i));
    end
    
    % =====================================================================
    %  BƯỚC 2: TÍNH ĐIỀU KHIỂN
    % =====================================================================
    u_total = zeros(3, n_drones);
    
    for i_drone = 1:n_drones
        u_form = zeros(3,1);
        gradV = zeros(3,1);
        dgradV = zeros(3,1);
        
        for e = 1:n_edges
            if edges(e,1) == i_drone
                j = edges(e,2);
                p_ij = pos(:, j) - pos(:, i_drone);
                v_ij = vel(:, j) - vel(:, i_drone);
                dij = norm(p_ij);
                
                if dij > 1e-6
                    e_ij = dij^2 - d_star(e)^2;
                    u_form = u_form + alpha_dist * e_ij * p_ij;
                    gradV = gradV + e_ij * p_ij;
                    term = 2 * (p_ij' * v_ij) * p_ij + e_ij * v_ij;
                    dgradV = dgradV + term;
                end
                
            elseif edges(e,2) == i_drone
                j = edges(e,1);
                p_ij = pos(:, j) - pos(:, i_drone);
                v_ij = vel(:, j) - vel(:, i_drone);
                dij = norm(p_ij);
                
                if dij > 1e-6
                    e_ij = dij^2 - d_star(e)^2;
                    u_form = u_form + alpha_dist * e_ij * p_ij;
                    gradV = gradV + e_ij * p_ij;
                    term = 2 * (p_ij' * v_ij) * p_ij + e_ij * v_ij;
                    dgradV = dgradV + term;
                end
            end
        end
        
        gradV = max(min(gradV, 15), -15);
        dgradV = max(min(dgradV, 20), -20);
        
        u_back = -kp * gradV - kd * dgradV;
        u_damp = -beta_dist * vel(:, i_drone);
        
        % Thành phần từ ước lượng vị trí
        u_pos = zeros(3,1);
        for e = 1:n_edges
            if edges(e,1) == i_drone
                j = edges(e,2);
                p_ij_est = pos_estimated(:, j) - pos_estimated(:, i_drone);
                p_ij_desired = (pes(:, j) - pes(:, i_drone));
                e_pos = p_ij_est - p_ij_desired;
                u_pos = u_pos + k_pos * e_pos;
            elseif edges(e,2) == i_drone
                j = edges(e,1);
                p_ij_est = pos_estimated(:, j) - pos_estimated(:, i_drone);
                p_ij_desired = (pes(:, j) - pes(:, i_drone));
                e_pos = p_ij_est - p_ij_desired;
                u_pos = u_pos + k_pos * e_pos;
            end
        end
        
        % Điều khiển độ cao
        u_height = zeros(3,1);
        k_height = 40;
        u_height(3) = -k_height * (pes(3, i_drone) - pos(3, i_drone));
        
        % Tổng hợp
        u_total(:, i_drone) = u_form + u_back + u_damp + 0.3 * u_pos + u_height;
    end
    
    % Thêm nhiễu
    u_total = u_total + noise_std * randn(3, n_drones);
    
    % =====================================================================
    %  BƯỚC 3: CẬP NHẬT ĐỘNG HỌC
    % =====================================================================
    acc = u_total;
    vel = vel + acc * dt;
    pos = pos + vel * dt;
    
    % Lưu lịch sử
    pos_history(:, :, step+1) = pos;
    vel_history(:, :, step+1) = vel;
    
    % Tính lỗi đồng thuận
    pos_center = mean(pos, 2);
    consensus_error(step) = mean(sqrt(sum((pos - pos_center).^2, 1)));
end

close(h);

%% ========================================================================
%  XỬ LÝ DỮ LIỆU SAU MÔ PHỎNG
% =========================================================================
% Lấy các vector có cùng kích thước
t_plot = t(1:N_steps-1);  % 0:dt:T_total-dt
error_consensus = consensus_error(1:N_steps-1);
error_estimation = estimation_error(1:N_steps-1, :);

% Kiểm tra kích thước
fprintf('\n=== KIỂM TRA KÍCH THƯỚC ===\n');
fprintf('t_plot: %d\n', length(t_plot));
fprintf('error_consensus: %d\n', length(error_consensus));
fprintf('error_estimation: %d x %d\n', size(error_estimation));

% Tính chỉ số
settling_idx = find(error_consensus < 0.5, 1, 'first');
if isempty(settling_idx)
    settling_time = T_total;
else
    settling_time = t_plot(settling_idx);
end

fprintf('\n=== KẾT THÚC MÔ PHỎNG ===\n');
fprintf('Thời gian ổn định (lỗi < 0.5m): %.2f s\n', settling_time);
fprintf('Lỗi đồng thuận cuối cùng: %.4f m\n', error_consensus(end));

% Tính lỗi ước lượng trong thời gian tấn công
idx_attack = t_plot >= attack_config.start_time & t_plot <= attack_config.end_time;
if any(idx_attack)
    mean_est_error = mean(mean(error_estimation(idx_attack, :)));
    fprintf('Lỗi ước lượng trung bình trong thời gian tấn công: %.4f m\n', mean_est_error);
else
    fprintf('Lỗi ước lượng trung bình trong thời gian tấn công: Không có dữ liệu\n');
end

%% ========================================================================
%  VẼ KẾT QUẢ
% =========================================================================
figure('Name', 'Phân tích ảnh hưởng của tấn công mạng', 'Position', [100, 100, 1400, 900]);

% Vùng tấn công
attack_region_x = [attack_config.start_time, attack_config.end_time, ...
                   attack_config.end_time, attack_config.start_time];

% Subplot 1: Lỗi đồng thuận
subplot(2,2,1);
plot(t_plot, error_consensus, 'b-', 'LineWidth', 1.5);
hold on;
fill(attack_region_x, [0, 0, 8, 8], 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
xlabel('Thời gian (s)');
ylabel('Lỗi đồng thuận (m)');
title('Lỗi đồng thuận (Consensus Error)');
grid on;
legend('Lỗi đồng thuận', 'Vùng tấn công', 'Location', 'best');
ylim([0, 8]);

% Subplot 2: Lỗi ước lượng
subplot(2,2,2);
colors = lines(n_drones);
for i = 1:n_drones
    plot(t_plot, error_estimation(:, i), 'Color', colors(i,:), 'LineWidth', 1);
    hold on;
end
fill(attack_region_x, [0, 0, 15, 15], 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
xlabel('Thời gian (s)');
ylabel('Lỗi ước lượng (m)');
title('Lỗi ước lượng vị trí từ mạng');
grid on;
ylim([0, 15]);

% Subplot 3: Quỹ đạo XY
subplot(2,2,3);
for i = 1:n_drones
    traj_x = squeeze(pos_history(1, i, :));
    traj_y = squeeze(pos_history(2, i, :));
    plot(traj_x, traj_y, 'Color', colors(i,:), 'LineWidth', 1);
    hold on;
    plot(traj_x(1), traj_y(1), 'o', 'Color', colors(i,:), 'MarkerSize', 6);
    plot(traj_x(end), traj_y(end), 's', 'Color', colors(i,:), 'MarkerSize', 6);
end
for i = 1:n_drones
    plot(pes(1,i), pes(2,i), '^', 'Color', colors(i,:), 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
end
xlabel('X (m)'); ylabel('Y (m)');
title('Quỹ đạo XY');
grid on; axis equal;

% Subplot 4: Độ cao theo thời gian
subplot(2,2,4);
for i = 1:n_drones
    traj_z = squeeze(pos_history(3, i, :));
    plot(t, traj_z, 'Color', colors(i,:), 'LineWidth', 1);
    hold on;
end
fill(attack_region_x, [0, 0, 12, 12], 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
xlabel('Thời gian (s)'); ylabel('Z (m)');
title('Độ cao theo thời gian');
grid on;
ylim([-2, 12]);

%% ========================================================================
%  Figure 2: So sánh trước và sau tấn công
% =========================================================================
if attack_scenario == 1
    figure('Name', 'So sánh trước và sau tấn công', 'Position', [100, 100, 1000, 600]);
    
    idx_before = t_plot < attack_config.start_time;
    idx_during = t_plot >= attack_config.start_time & t_plot < attack_config.end_time;
    idx_after = t_plot >= attack_config.end_time;
    
    error_before = error_consensus(idx_before);
    error_during = error_consensus(idx_during);
    error_after = error_consensus(idx_after);
    
    % Bar chart
    subplot(1,2,1);
    bar_data = [mean(error_before), mean(error_during), mean(error_after)];
    bar(bar_data);
    set(gca, 'XTickLabel', {'Trước tấn công', 'Trong tấn công', 'Sau tấn công'});
    ylabel('Lỗi đồng thuận trung bình (m)');
    title('So sánh lỗi đồng thuận');
    grid on;
    
    % Tốc độ hội tụ
    subplot(1,2,2);
    derror = diff(error_consensus) / dt;
    plot(t_plot(2:end), derror, 'b-', 'LineWidth', 1);
    hold on;
    fill(attack_region_x, [-5, -5, 5, 5], 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    xlabel('Thời gian (s)');
    ylabel('Tốc độ hội tụ (m/s)');
    title('Tốc độ hội tụ (đạo hàm lỗi)');
    grid on;
    ylim([-2, 2]);
    
    % In thống kê
    fprintf('\n=== THỐNG KÊ ===\n');
    if ~isempty(error_before)
        fprintf('Lỗi trung bình trước tấn công: %.4f m\n', mean(error_before));
    end
    if ~isempty(error_during)
        fprintf('Lỗi trung bình trong tấn công: %.4f m\n', mean(error_during));
    end
    if ~isempty(error_after)
        fprintf('Lỗi trung bình sau tấn công: %.4f m\n', mean(error_after));
    end
    if ~isempty(error_before) && ~isempty(error_during) && mean(error_before) > 0
        increase_pct = (mean(error_during)/mean(error_before)-1)*100;
        fprintf('Tăng lỗi khi bị tấn công: %.1f%%\n', increase_pct);
    end
end

fprintf('\n=== MÔ PHỎNG HOÀN TẤT ===\n');