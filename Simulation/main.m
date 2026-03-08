%% ĐỒ ÁN: ĐIỀU KHIỂN ĐỘI HÌNH DRONE 3D
% Tính năng: 
% - Bám quỹ đạo vòng tròn
% - Giữ liên kết với neighbors
% - Tránh va chạm
% - Switching topology khi mất kết nối

clear; clc; close all;

%% Tham số mô phỏng
dt = 0.01;          % bước thời gian (s)
T = 60;             % thời gian mô phỏng (s)
t = 0:dt:T;
n = 4;              % số drone
d = 3;              % không gian 3D

%% Tham số điều khiển
% Tầng 1: Vị trí tuyệt đối
k_p = 1.5;          % hệ số bám quỹ đạo
k_f = 0.8;          % hệ số giữ đội hình

% Tầng 2: Giữ liên kết
d_comm_max = 15;    % khoảng cách tối đa để giữ liên lạc (m)
k_connect = 0.5;    % hệ số kéo về để giữ liên kết

% Tầng 3: Tránh va chạm
d_safe = 2.5;       % khoảng cách an toàn tối thiểu (m)
d_detection = 5;    % khoảng cách phát hiện va chạm (m)
k_repulse = 2.0;    % hệ số đẩy khi tránh va chạm

% Tầng 4: Switching topology
topology_update_rate = 0.5;  % cập nhật topology mỗi 0.5s
last_topology_update = 0;

%% Định nghĩa đội hình (hình vuông cạnh 3m)
L = 3;
formation_offset = [0, 0, 0;
                    L, 0, 0;
                    L, L, 0;
                    0, L, 0];

%% Định nghĩa quỹ đạo cho tâm đội hình
R_orbit = 20;               % bán kính quỹ đạo
omega = 0.15;               % tốc độ góc
z_base = 8;                 % độ cao cơ bản
z_amplitude = 3;            % biên độ dao động độ cao

% Quỹ đạo tâm: vòng tròn + lên xuống hình sin
p0_star = @(t) [R_orbit * cos(omega*t);
                R_orbit * sin(omega*t);
                z_base + z_amplitude * sin(0.2*t)];

%% Vị trí ban đầu
p0 = [25; 5; 10;     % drone 1
      15; 20; 7;     % drone 2
      5; 10; 13;     % drone 3
      18; 12; 5];    % drone 4

%% Khởi tạo ma trận kết nối ban đầu (đồ thị đầy đủ K4)
adj = ones(n) - eye(n);
L_laplacian = diag(sum(adj,2)) - adj;

%% Mô phỏng chính
p = p0;
v = zeros(n, d);
p_history = zeros(length(t), n*d);
adj_history = zeros(length(t), n, n);
collision_events = [];
disconnection_events = [];

fprintf('=== BẮT ĐẦU MÔ PHỎNG ===\n');
fprintf('Thời gian: %d giây\n', T);

for k = 1:length(t)
    current_time = t(k);
    p_history(k,:) = p';
    
    % TÁCH VỊ TRÍ TỪNG DRONE
    p_mat = reshape(p, [d, n])';
    
    % TẦNG 4: CẬP NHẬT TOPOLOGY (theo thời gian và khoảng cách)
    if current_time - last_topology_update >= topology_update_rate
        [adj_new, disconnections] = update_topology(p_mat, d_comm_max, adj);
        adj = adj_new;
        L_laplacian = diag(sum(adj,2)) - adj;
        last_topology_update = current_time;
        
        % Ghi nhận sự kiện mất kết nối
        if disconnections > 0
            disconnection_events = [disconnection_events; current_time, disconnections];
        end
    end
    adj_history(k,:,:) = adj;
    
    % TÍNH QUỸ ĐẠO MONG MUỐN CHO TỪNG DRONE
    p0_current = p0_star(current_time);
    p_star_mat = p0_current' + formation_offset;
    p_star = p_star_mat(:);
    
    % TẦNG 1: LỰC ĐIỀU KHIỂN VỊ TRÍ TUYỆT ĐỐI (LUẬT 6.5)
    delta = p_star - p;
    F_position = k_p * delta - kron(L_laplacian, eye(d)) * (k_f * delta);
    F_position = reshape(F_position, [d, n])';
    
    % TẦNG 2: LỰC GIỮ LIÊN KẾT (kéo về khi sắp mất kết nối)
    F_connect = zeros(n, d);
    for i = 1:n
        for j = 1:n
            if adj(i,j) > 0 && i ~= j
                vec_ij = p_mat(j,:) - p_mat(i,:);
                dist = norm(vec_ij);
                
                % Nếu sắp vượt quá khoảng cách cho phép
                if dist > 0.8 * d_comm_max
                    % Kéo về hướng trung tâm
                    dir = vec_ij / dist;
                    F_connect(i,:) = F_connect(i,:) + k_connect * (dist - d_comm_max) * dir;
                end
            end
        end
    end
    
    % TẦNG 3: LỰC TRÁNH VA CHẠM
    F_repulse = zeros(n, d);
    for i = 1:n
        for j = i+1:n
            vec_ij = p_mat(j,:) - p_mat(i,:);
            dist = norm(vec_ij);
            
            if dist < d_detection && dist > 0.01
                % Lực đẩy tỷ lệ nghịch với khoảng cách
                dir_ij = vec_ij / dist;
                magnitude = k_repulse * (1/dist - 1/d_detection) / dist^2;
                
                F_repulse(i,:) = F_repulse(i,:) - magnitude * dir_ij;
                F_repulse(j,:) = F_repulse(j,:) + magnitude * dir_ij;
                
                % Ghi nhận sự kiện va chạm
                if dist < d_safe
                    collision_events = [collision_events; current_time, i, j, dist];
                end
            end
        end
    end
    
    % TỔNG HỢP LỰC
    F_total = F_position + F_connect + F_repulse;
    
    % GIỚI HẠN LỰC (tránh sốc)
    max_force = 8.0;
    for i = 1:n
        force_mag = norm(F_total(i,:));
        if force_mag > max_force
            F_total(i,:) = F_total(i,:) * (max_force / force_mag);
        end
    end
    
    % CẬP NHẬT VẬN TỐC VÀ VỊ TRÍ
    v = v + dt * F_total;
    
    % GIỚI HẠN VẬN TỐC
    max_vel = 5.0;
    for i = 1:n
        vel_mag = norm(v(i,:));
        if vel_mag > max_vel
            v(i,:) = v(i,:) * (max_vel / vel_mag);
        end
    end
    
    p = p + dt * v(:);
    
    % In tiến trình
    if mod(k, round(length(t)/10)) == 0
        fprintf('t = %.1f, số kết nối: %d\n', current_time, sum(adj(:))/2);
    end
end

%% VẼ KẾT QUẢ
plot_results(p_history, adj_history, p_star_mat, t, collision_events, disconnection_events);

%% ==================== HÀM HỖ TRỢ ====================

function [adj_new, disconnections] = update_topology(p_mat, d_max, adj_old)
% Cập nhật topology dựa trên khoảng cách thực tế
    n = size(p_mat,1);
    adj_new = zeros(n);
    disconnections = 0;
    
    for i = 1:n
        for j = i+1:n
            dist = norm(p_mat(i,:) - p_mat(j,:));
            if dist < d_max
                adj_new(i,j) = 1;
                adj_new(j,i) = 1;
                
                % Kiểm tra mất kết nối
                if adj_old(i,j) == 0
                    % Kết nối mới
                end
            else
                if adj_old(i,j) == 1
                    disconnections = disconnections + 1;
                end
            end
        end
    end
end

function plot_results(p_history, adj_history, p_star_mat, t, collision_events, disconnection_events)
% Vẽ kết quả mô phỏng
    figure('Position', [50 50 1600 900]);
    
    % Lấy kích thước
    [n_time, n_total] = size(p_history);
    n = 4; d = 3;
    
    % Chuyển về dạng mảng
    p_reshaped = reshape(p_history, [n_time, n, d]);
    
    % 1. QUỸ ĐẠO 3D
    subplot(2,3,1);
    colors = {'r', 'b', 'g', 'm'};
    for i = 1:n
        x = squeeze(p_reshaped(:,i,1));
        y = squeeze(p_reshaped(:,i,2));
        z = squeeze(p_reshaped(:,i,3));
        plot3(x, y, z, colors{i}, 'LineWidth', 1.5); hold on
        plot3(x(1), y(1), z(1), [colors{i},'o'], 'MarkerSize', 8);
        plot3(x(end), y(end), z(end), [colors{i},'s'], 'MarkerSize', 8);
    end
    
    % Quỹ đạo tâm mong muốn
    t_plot = linspace(0, t(end), 100);
    R = 20; omega = 0.15; z0 = 8; z_amp = 3;
    x0 = R * cos(omega*t_plot);
    y0 = R * sin(omega*t_plot);
    z0_plot = z0 + z_amp * sin(0.2*t_plot);
    plot3(x0, y0, z0_plot, 'k--', 'LineWidth', 2);
    
    xlabel('x'); ylabel('y'); zlabel('z');
    title('Quỹ đạo 3D');
    legend('Drone1','Drone2','Drone3','Drone4','Target');
    grid on; view(45,30);
    
    % 2. KHOẢNG CÁCH GIỮA CÁC DRONE
    subplot(2,3,2);
    distances = [];
    pairs = [1,2; 1,3; 1,4; 2,3; 2,4; 3,4];
    for p = 1:size(pairs,1)
        i = pairs(p,1); j = pairs(p,2);
        d_ij = squeeze(sqrt(sum((p_reshaped(:,i,:) - p_reshaped(:,j,:)).^2, 3)));
        plot(t, d_ij, 'LineWidth', 1); hold on
        distances = [distances, d_ij];
    end
    yline(2.5, 'r--', 'Safe distance');
    xlabel('t (s)'); ylabel('Khoảng cách (m)');
    title('Khoảng cách giữa các drone');
    legend('1-2','1-3','1-4','2-3','2-4','3-4');
    grid on;
    
    % 3. SỐ LƯỢNG KẾT NỐI
    subplot(2,3,3);
    n_connections = squeeze(sum(sum(adj_history,3),2))/2;
    plot(t, n_connections, 'b-', 'LineWidth', 2);
    xlabel('t (s)'); ylabel('Số kết nối');
    title('Số lượng kết nối');
    grid on;
    ylim([0, 6]);
    
    % 4. SỰ KIỆN MẤT KẾT NỐI
    subplot(2,3,4);
    if ~isempty(disconnection_events)
        stem(disconnection_events(:,1), disconnection_events(:,2), 'r', 'LineWidth', 2);
    end
    xlabel('t (s)'); ylabel('Số lần mất kết nối');
    title('Sự kiện mất kết nối');
    grid on;
    xlim([0, t(end)]);
    
    % 5. SỰ KIỆN VA CHẠM
    subplot(2,3,5);
    if ~isempty(collision_events)
        scatter(collision_events(:,1), collision_events(:,4), 50, 'r', 'filled'); hold on
    end
    xlabel('t (s)'); ylabel('Khoảng cách khi va chạm (m)');
    title('Sự kiện va chạm');
    yline(2.5, 'r--');
    grid on;
    xlim([0, t(end)]);
    
    % 6. SAI SỐ BÁM QUỸ ĐẠO
    subplot(2,3,6);
    tracking_error = zeros(n_time, n);
    for i = 1:n
        p_star_i = p_star_mat(i,:);
        for k = 1:n_time
            tracking_error(k,i) = norm(squeeze(p_reshaped(k,i,:))' - p_star_i);
        end
    end
    plot(t, tracking_error, 'LineWidth', 1.5);
    xlabel('t (s)'); ylabel('Sai số (m)');
    title('Sai số bám quỹ đạo');
    legend('Drone1','Drone2','Drone3','Drone4');
    grid on;
    
    % Thông tin thống kê
    fprintf('\n=== THỐNG KÊ ===\n');
    fprintf('Tổng số lần mất kết nối: %d\n', size(disconnection_events,1));
    fprintf('Tổng số lần va chạm: %d\n', size(collision_events,1));
    fprintf('Sai số bám trung bình: %.3f m\n', mean(tracking_error(:)));
end