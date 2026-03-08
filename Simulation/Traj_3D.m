%% Mô phỏng 5 drone trong không gian 3D - Quỹ đạo xoắn ốc đơn giản
% Drone 1: bám leader (offset [0;0;0])
% 4 drone còn lại: 4 vị trí trong không gian cục bộ
% Chỉ dùng góc yaw (psi) để xoay offset

clear all; close all; clc;

%% 1. THAM SỐ MÔ PHỎNG
dt = 0.05;
T = 60;
t = 0:dt:T;
n_steps = length(t);

% Tham số quỹ đạo xoắn ốc (spiral)
R = 30;                 % bán kính xoay
omega_xy = 0.2;         % tốc độ góc trong mặt phẳng XY
vz = 0.5;               % vận tốc leo theo trục Z (m/s)
z0 = 10;                % độ cao ban đầu

%% 2. TẠO QUỸ ĐẠO LEADER ẢO (3D - xoắn ốc đơn giản)
% Vị trí leader
x_leader = R * cos(omega_xy * t);
y_leader = R * sin(omega_xy * t);
z_leader = z0 + vz * t;  % tăng đều theo thời gian

% Vận tốc leader
vx_leader = -R * omega_xy * sin(omega_xy * t);
vy_leader = R * omega_xy * cos(omega_xy * t);
vz_leader = vz * ones(1, n_steps);

% Góc yaw (psi) - góc xoay quanh trục Z
% Với quỹ đạo tròn: psi = omega_xy*t + pi/2
psi_leader = atan2(y_leader, x_leader);

p_leader = [x_leader; y_leader; z_leader];

%% 3. OFFSET TRONG HỆ TỌA ĐỘ GẮN VỚI LEADER (body frame)
% Tạo 5 điểm trong không gian cục bộ:
% Drone 1: [0;0;0] - trùng leader
% Drone 2: [offset; 0; 0] - bên phải
% Drone 3: [-offset; 0; 0] - bên trái
% Drone 4: [0; offset; 0] - phía trước
% Drone 5: [0; -offset; 0] - phía sau

offset = 5;  % khoảng cách offset
offsets_body = [0, 0, 0;
                offset, 0, 0;
                -offset, 0, 0;
                0, offset, 0;
                0, -offset, 0]';  % ma trận 3x5

N = 5;  % số drone

% Tên drone cho hiển thị
drone_names = {'Drone 1 (trùng)', 'Drone 2 (phải)', 'Drone 3 (trái)', ...
               'Drone 4 (trước)', 'Drone 5 (sau)'};

%% 4. TÍNH VỊ TRÍ MONG MUỐN p_star(i) = p_leader + R(psi) * offset_body(i)
p_star = zeros(3, N, n_steps);

for k = 1:n_steps
    psi = psi_leader(k);
    
    % Ma trận quay quanh trục Z (chỉ dùng yaw)
    % Từ body frame sang world frame
    R_psi = [cos(psi), -sin(psi), 0;
             sin(psi), cos(psi), 0;
             0, 0, 1];
    
    for i = 1:N
        % Xoay offset từ body frame sang world frame
        offset_world = R_psi * offsets_body(:,i);
        
        % Vị trí mong muốn = vị trí leader + offset đã xoay
        p_star(:,i,k) = p_leader(:,k) + offset_world;
    end
end

%% 5. KIỂM TRA KHOẢNG CÁCH
fprintf('=== KIỂM TRA KHOẢNG CÁCH ===\n');
for i = 1:N
    dist_to_leader = zeros(1, n_steps);
    for k = 1:n_steps
        dist_to_leader(k) = norm(p_star(:,i,k) - p_leader(:,k));
    end
    fprintf('Drone %d: Khoảng cách đến leader = %.2f ± %.2f m\n', ...
            i, mean(dist_to_leader), std(dist_to_leader));
end

%% 6. THIẾT LẬP MA TRẬN KẾT NỐI
A_adj = ones(N,N) - eye(N);  % fully connected
a_ij = 0.5;

% Ma trận Laplacian
L = zeros(N,N);
for i = 1:N
    for j = 1:N
        if i ~= j && A_adj(i,j) == 1
            L(i,i) = L(i,i) + a_ij;
            L(i,j) = -a_ij;
        end
    end
end

%% 7. THAM SỐ ĐIỀU KHIỂN
kp = 2.0;    % hệ số bám vị trí

%% 8. KHỞI TẠO VỊ TRÍ DRONE (có sai số)
rng(42);
p_drone = zeros(3, N, n_steps);
for i = 1:N
    p_drone(:,i,1) = p_star(:,i,1) + 3 * randn(3,1);
end

% Drone 1 bắt đầu gần leader hơn
p_drone(:,1,1) = p_leader(:,1) + 0.5 * randn(3,1);

%% 9. VÒNG LẬP MÔ PHỎNG - CÔNG THỨC (6.5) trong 3D
fprintf('Đang mô phỏng 5 drone trong không gian 3D...\n');

for k = 1:n_steps-1
    p_current = p_drone(:,:,k);
    p_star_k = p_star(:,:,k);
    
    % Sai số vị trí e_i = p_i^* - p_i
    e_pos = p_star_k - p_current;
    
    % Tính u theo công thức (6.5)
    u = zeros(3, N);
    
    for i = 1:N
        % Thành phần bám vị trí
        u(:,i) = kp * e_pos(:,i);
        
        % Thành phần đồng thuận
        for j = 1:N
            if i ~= j
                u(:,i) = u(:,i) - a_ij * (e_pos(:,j) - e_pos(:,i));
            end
        end
    end
    
    % Cập nhật vị trí
    p_drone(:,:,k+1) = p_current + dt * u;
end

%% 10. VẼ KẾT QUẢ 3D
figure('Position', [50, 50, 1400, 900]);

% Plot 1: Quỹ đạo 3D
subplot(2,3,[1,2,4,5]);
hold on;

% Vẽ quỹ đạo leader
plot3(p_leader(1,:), p_leader(2,:), p_leader(3,:), 'k-', 'LineWidth', 3, 'DisplayName', 'Leader');

% Vẽ quỹ đạo các drone
colors = lines(N);
for i = 1:N
    plot3(squeeze(p_drone(1,i,:)), squeeze(p_drone(2,i,:)), squeeze(p_drone(3,i,:)), ...
          'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', drone_names{i});
end

% Đánh dấu điểm đầu
plot3(p_leader(1,1), p_leader(2,1), p_leader(3,1), 'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
for i = 1:N
    plot3(p_drone(1,i,1), p_drone(2,i,1), p_drone(3,i,1), 'o', 'Color', colors(i,:), ...
          'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
end

% Đánh dấu điểm cuối
plot3(p_leader(1,end), p_leader(2,end), p_leader(3,end), 'kd', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
for i = 1:N
    plot3(p_drone(1,i,end), p_drone(2,i,end), p_drone(3,i,end), 's', 'Color', colors(i,:), ...
          'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
end

xlabel('x (m)', 'FontSize', 12);
ylabel('y (m)', 'FontSize', 12);
zlabel('z (m)', 'FontSize', 12);
title('Quỹ đạo 3D: Leader và 5 drone (xoắn ốc đơn giản)', 'FontSize', 14);
legend('Location', 'best');
grid on;
view(45, 30);  % góc nhìn 3D

% Plot 2: Hình chiếu XY
subplot(2,3,3);
hold on;
plot(p_leader(1,:), p_leader(2,:), 'k-', 'LineWidth', 2, 'DisplayName', 'Leader');
for i = 1:N
    plot(squeeze(p_drone(1,i,:)), squeeze(p_drone(2,i,:)), 'Color', colors(i,:), 'LineWidth', 1.5);
end
xlabel('x (m)'); ylabel('y (m)');
title('Hình chiếu XY');
axis equal; grid on;
legend('Location', 'best');

% Plot 3: Sai số bám
subplot(2,3,6);
hold on;
for i = 1:N
    error = zeros(1, n_steps);
    for k = 1:n_steps
        error(k) = norm(p_star(:,i,k) - p_drone(:,i,k));
    end
    semilogy(t, error, 'Color', colors(i,:), 'LineWidth', 1.5);
end
xlabel('Thời gian (s)', 'FontSize', 12);
ylabel('Sai số (m)', 'FontSize', 12);
title('Sai số bám vị trí', 'FontSize', 12);
grid on;
legend(drone_names, 'Location', 'best');

sgtitle('Mô phỏng 5 drone trong không gian 3D - Quỹ đạo xoắn ốc', 'FontSize', 16);

%% 11. TÍNH TOÁN SAI SỐ CUỐI CÙNG
fprintf('\n=== SAI SỐ CUỐI CÙNG (t = %.1f s) ===\n', T);
for i = 1:N
    error_final = norm(p_star(:,i,end) - p_drone(:,i,end));
    fprintf('Drone %d: %.4f m\n', i, error_final);
end

%% 12. VẼ HÌNH MINH HỌA FORMATION TẠI MỘT SỐ THỜI ĐIỂM
figure('Position', [150, 150, 1200, 400]);

snapshot_times = [0, 20, 40, 60];
for s = 1:4
    subplot(1,4,s);
    hold on;
    
    [~, idx] = min(abs(t - snapshot_times(s)));
    
    % Vẽ leader
    plot3(p_leader(1,idx), p_leader(2,idx), p_leader(3,idx), 'ks', ...
          'MarkerSize', 15, 'MarkerFaceColor', 'k');
    
    % Vẽ các drone
    for i = 1:N
        plot3(p_drone(1,i,idx), p_drone(2,i,idx), p_drone(3,i,idx), 'o', ...
              'Color', colors(i,:), 'MarkerSize', 10, 'MarkerFaceColor', colors(i,:));
    end
    
    % Vẽ kết nối giữa các drone (tạo thành hình chữ thập)
    % Nối drone 2-3 (phải-trái)
    plot3([p_drone(1,2,idx), p_drone(1,3,idx)], ...
          [p_drone(2,2,idx), p_drone(2,3,idx)], ...
          [p_drone(3,2,idx), p_drone(3,3,idx)], 'k--', 'LineWidth', 0.5);
    
    % Nối drone 4-5 (trước-sau)
    plot3([p_drone(1,4,idx), p_drone(1,5,idx)], ...
          [p_drone(2,4,idx), p_drone(2,5,idx)], ...
          [p_drone(3,4,idx), p_drone(3,5,idx)], 'k--', 'LineWidth', 0.5);
    
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    title(sprintf('t = %.1f s', t(idx)));
    grid on;
    view(45, 30);
    xlim([-40, 40]); ylim([-40, 40]); zlim([0, 50]);
end
sgtitle('Formation tại các thời điểm', 'FontSize', 14);

%% 13. KẾT LUẬN
fprintf('\n=== GIẢI THÍCH ===\n');
fprintf('1. Leader bay theo quỹ đạo xoắn ốc: x = Rcos(ωt), y = Rsin(ωt), z = z0 + v_z*t\n');
fprintf('2. Vị trí drone i: p_i = p_leader + R(psi) * offset_i\n');
fprintf('3. R(psi) là ma trận xoay quanh trục Z với góc psi = atan2(vy, vx)\n');
fprintf('4. Offsets trong body frame:\n');
for i = 1:N
    fprintf('   %s: [%.1f, %.1f, %.1f]\n', drone_names{i}, offsets_body(1,i), offsets_body(2,i), offsets_body(3,i));
end
fprintf('5. Luật điều khiển: u_i = k_p(p_i^* - p_i) + Σa_ij((p_j^*-p_j) - (p_i^*-p_i))\n');