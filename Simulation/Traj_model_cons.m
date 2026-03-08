%% Mô phỏng 5 drone với mô hình bậc 2: p_ddot = u
% Drone 1: bám leader (offset [0;0;0])
% 4 drone còn lại: 4 vị trí trong không gian cục bộ

clear all; close all; clc;

%% 1. THAM SỐ MÔ PHỎNG
dt = 0.02;               % bước thời gian nhỏ hơn cho hệ bậc 2
T = 100;
t = 0:dt:T;
n_steps = length(t);

% Tham số quỹ đạo xoắn ốc
R = 30;                  % bán kính
omega = 0.2;             % tốc độ góc
vz = 0.5;                % vận tốc leo
z0 = 10;                 % độ cao ban đầu

%% 2. TẠO QUỸ ĐẠO LEADER (và đạo hàm cấp 1, cấp 2)
% Vị trí
x_leader = R * cos(omega * t);
y_leader = R * sin(omega * t);
z_leader = z0 + vz * t;

% Vận tốc (đạo hàm cấp 1)
vx_leader = -R * omega * sin(omega * t);
vy_leader = R * omega * cos(omega * t);
vz_leader = vz * ones(1, n_steps);

% Gia tốc (đạo hàm cấp 2)
ax_leader = -R * omega^2 * cos(omega * t);
ay_leader = -R * omega^2 * sin(omega * t);
az_leader = zeros(1, n_steps);

% Góc yaw
psi_leader = atan2(y_leader, x_leader);

p_leader = [x_leader; y_leader; z_leader];
v_leader = [vx_leader; vy_leader; vz_leader];
a_leader = [ax_leader; ay_leader; az_leader];

%% 3. OFFSET TRONG BODY FRAME
offset = 5;
offsets_body = [0, 0, 0;
                offset, 0, 0;
                -offset, 0, 0;
                0, offset, 0;
                0, -offset, 0]';  % 3x5

N = 5;
drone_names = {'Drone 1 (trùng)', 'Drone 2 (phải)', 'Drone 3 (trái)', ...
               'Drone 4 (trước)', 'Drone 5 (sau)'};

%% 4. TÍNH QUỸ ĐẠO MONG MUỐN (p*, v*, a*)
p_star = zeros(3, N, n_steps);
v_star = zeros(3, N, n_steps);
a_star = zeros(3, N, n_steps);

for k = 1:n_steps
    psi = psi_leader(k);
    dpsi = omega;  % đạo hàm của psi (với quỹ đạo tròn)
    
    % Ma trận xoay và đạo hàm của nó
    R_psi = [cos(psi), -sin(psi), 0;
             sin(psi), cos(psi), 0;
             0, 0, 1];
    
    R_dot = dpsi * [-sin(psi), -cos(psi), 0;
                     cos(psi), -sin(psi), 0;
                     0, 0, 0];
    
    R_ddot = dpsi^2 * [-cos(psi), sin(psi), 0;
                        -sin(psi), -cos(psi), 0;
                        0, 0, 0];
    
    for i = 1:N
        offset_body = offsets_body(:,i);
        
        % Vị trí mong muốn
        offset_world = R_psi * offset_body;
        p_star(:,i,k) = p_leader(:,k) + offset_world;
        
        % Vận tốc mong muốn (đạo hàm)
        offset_world_dot = R_dot * offset_body;
        v_star(:,i,k) = v_leader(:,k) + offset_world_dot;
        
        % Gia tốc mong muốn (đạo hàm cấp 2)
        offset_world_ddot = R_ddot * offset_body;
        a_star(:,i,k) = a_leader(:,k) + offset_world_ddot;
    end
end

%% 5. THIẾT LẬP MA TRẬN KẾT NỐI
A_adj = ones(N,N) - eye(N);
a_ij = 0.5;

global L;
L = zeros(N,N);
for i = 1:N
    for j = 1:N
        if i ~= j && A_adj(i,j) == 1
            L(i,i) = L(i,i) + a_ij;
            L(i,j) = -a_ij;
        end
    end
end

%% 6. THAM SỐ ĐIỀU KHIỂN CHO HỆ BẬC 2
kp = 4.0;    % hệ số P (vị trí)
kd = 3.0;    % hệ số D (vận tốc)

%% 7. KHỞI TẠO TRẠNG THÁI DRONE
p_drone = zeros(3, N, n_steps);
v_drone = zeros(3, N, n_steps);

rng(42);
for i = 1:N
    p_drone(:,i,1) = p_star(:,i,1) + 3 * randn(3,1);
    v_drone(:,i,1) = v_star(:,i,1) + 0.5 * randn(3,1);
end

% Drone 1 gần leader hơn
p_drone(:,1,1) = p_leader(:,1) + 0.5 * randn(3,1);

%% 8. VÒNG LẬP MÔ PHỎNG - HỆ BẬC 2
fprintf('Đang mô phỏng hệ bậc 2 (p_ddot = u)...\n');

for k = 1:n_steps-1
    p_current = p_drone(:,:,k);
    v_current = v_drone(:,:,k);
    
    p_star_k = p_star(:,:,k);
    v_star_k = v_star(:,:,k);
    a_star_k = a_star(:,:,k);
    
    % Sai số
    e_p = p_star_k - p_current;
    e_v = v_star_k - v_current;
    
    % Tính u (gia tốc điều khiển)
    u = zeros(3, N);
    
    for i = 1:N
        % PD + feedforward
        u(:,i) = kp * e_p(:,i) + kd * e_v(:,i) + a_star_k(:,i);
        
        % Consensus trên sai số
        for j = 1:N
            if i ~= j
                u(:,i) = u(:,i) - a_ij * ((e_p(:,j) - e_p(:,i)) + (e_v(:,j) - e_v(:,i)));
            end
        end
    end
    
    % Tích phân: cập nhật vận tốc và vị trí
    v_drone(:,:,k+1) = v_current + dt * u;
    p_drone(:,:,k+1) = p_current + dt * v_current + 0.5 * dt^2 * u;
end

%% 9. VẼ KẾT QUẢ
figure('Position', [50, 50, 1600, 900]);

% Plot 1: Quỹ đạo 3D
subplot(2,4,[1,2,5,6]);
hold on;

% Vẽ quỹ đạo leader
plot3(p_leader(1,:), p_leader(2,:), p_leader(3,:), 'k-', 'LineWidth', 3, 'DisplayName', 'Leader');

% Vẽ quỹ đạo drone
colors = lines(N);
for i = 1:N
    plot3(squeeze(p_drone(1,i,:)), squeeze(p_drone(2,i,:)), squeeze(p_drone(3,i,:)), ...
          'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', drone_names{i});
end

% Điểm đầu
plot3(p_leader(1,1), p_leader(2,1), p_leader(3,1), 'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
for i = 1:N
    plot3(p_drone(1,i,1), p_drone(2,i,1), p_drone(3,i,1), 'o', 'Color', colors(i,:), ...
          'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
end

xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('Quỹ đạo 3D - Hệ bậc 2: \ddot{p} = u');
legend('Location', 'best');
grid on; view(45, 30);

% Plot 2: Sai số vị trí
subplot(2,4,3);
hold on;
for i = 1:N
    error_p = zeros(1, n_steps);
    for k = 1:n_steps
        error_p(k) = norm(p_star(:,i,k) - p_drone(:,i,k));
    end
    semilogy(t, error_p, 'Color', colors(i,:), 'LineWidth', 1.5);
end
xlabel('Thời gian (s)'); ylabel('Sai số (m)');
title('Sai số vị trí');
grid on; legend(drone_names, 'Location', 'best');

% Plot 3: Sai số vận tốc
subplot(2,4,4);
hold on;
for i = 1:N
    error_v = zeros(1, n_steps);
    for k = 1:n_steps
        error_v(k) = norm(v_star(:,i,k) - v_drone(:,i,k));
    end
    semilogy(t, error_v, 'Color', colors(i,:), 'LineWidth', 1.5);
end
xlabel('Thời gian (s)'); ylabel('Sai số (m/s)');
title('Sai số vận tốc');
grid on; legend(drone_names, 'Location', 'best');

% Plot 4: Gia tốc u (minh họa cho drone 1)
subplot(2,4,7);
hold on;
u_drone1 = diff(v_drone(:,1,:), 1, 3) / dt;
u_drone1 = squeeze(u_drone1);
plot(t(1:end-1), u_drone1(1,:), 'r-', 'LineWidth', 1);
plot(t(1:end-1), u_drone1(2,:), 'g-', 'LineWidth', 1);
plot(t(1:end-1), u_drone1(3,:), 'b-', 'LineWidth', 1);
xlabel('Thời gian (s)'); ylabel('Gia tốc (m/s^2)');
title('Gia tốc điều khiển u (drone 1)');
legend({'u_x', 'u_y', 'u_z'});
grid on;

% Plot 5: Khoảng cách giữa các drone
subplot(2,4,8);
hold on;
dist_pairs = {[1,2], [1,3], [1,4], [1,5]};
for p = 1:4
    i = dist_pairs{p}(1);
    j = dist_pairs{p}(2);
    dist = zeros(1, n_steps);
    for k = 1:n_steps
        dist(k) = norm(p_drone(:,i,k) - p_drone(:,j,k));
    end
    plot(t, dist, 'LineWidth', 1.5, 'DisplayName', sprintf('Drone %d-%d', i, j));
end
% Khoảng cách mong muốn
yline(offset, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Mong muốn');
xlabel('Thời gian (s)'); ylabel('Khoảng cách (m)');
title('Khoảng cách giữa các drone');
legend('Location', 'best');
grid on;

sgtitle('Mô phỏng hệ bậc 2 với 5 drone', 'FontSize', 16);

%% 10. SO SÁNH VỚI HỆ BẬC 1
fprintf('\n=== SO SÁNH VỚI HỆ BẬC 1 ===\n');
fprintf('Hệ bậc 1 (p_dot = u):\n');
fprintf('  - Điều khiển vận tốc trực tiếp\n');
fprintf('  - Đáp ứng tức thời, không quán tính\n');
fprintf('  - Dễ implement nhưng ít thực tế\n\n');

fprintf('Hệ bậc 2 (p_ddot = u):\n');
fprintf('  - Điều khiển gia tốc\n');
fprintf('  - Có quán tính, thực tế hơn\n');
fprintf('  - Cần thêm thành phần D (vận tốc) trong luật điều khiển\n');
fprintf('  - Cần đạo hàm cấp 1,2 của quỹ đạo mong muốn\n');

%% 11. TÍNH TOÁN THỐNG KÊ
fprintf('\n=== SAI SỐ CUỐI CÙNG (t = %.1f s) ===\n', T);
for i = 1:N
    error_p_final = norm(p_star(:,i,end) - p_drone(:,i,end));
    error_v_final = norm(v_star(:,i,end) - v_drone(:,i,end));
    fprintf('Drone %d: e_p = %.4f m, e_v = %.4f m/s\n', i, error_p_final, error_v_final);
end