%% Mô phỏng điều khiển đội hình dựa trên vị trí tuyệt đối - 3D
% Luật (6.5): ṗ_i = k_p(p_i^* - p_i) + Σ a_ij[(p_j^* - p_j) - (p_i^* - p_i)]
% Dạng ma trận (6.6): ṗ = k_p(p^* - p) - (L ⊗ I)(p^* - p)

clear; clc; close all;

%% THAM SỐ MÔ PHỎNG
dt = 0.01;              % bước thời gian (s)
T = 15;                 % thời gian mô phỏng (s) - nhanh hơn vì luật này ổn định
t = 0:dt:T;
n = 4;                  % số drone
d = 3;                  % không gian 3D

% Hệ số điều khiển
k_p = 1.0;              % hệ số tỷ lệ (có thể tăng/giảm)

%% ĐỊNH NGHĨA ĐỘI HÌNH HÌNH VUÔNG TRONG 3D
% Đặt hình vuông trong mặt phẳng z = 2
z0 = 2;  % độ cao mong muốn
L = 3;   % cạnh hình vuông

p_star = [0, 0, z0;      % drone 1 - góc dưới trái
          L, 0, z0;      % drone 2 - góc dưới phải
          L, L, z0;      % drone 3 - góc trên phải
          0, L, z0];     % drone 4 - góc trên trái

%% ĐỒ THỊ KẾT NỐI (đồ thị đầy đủ K4)
% Ma trận trọng số a_ij = 1 cho tất cả các cạnh
adj = ones(n) - eye(n);  % ma trận kề (không có tự kết nối)

% Ma trận Laplace
deg = diag(sum(adj, 2));
L_laplacian = deg - adj;

%% VỊ TRÍ BAN ĐẦU - ngẫu nhiên trong không gian 3D
rng(42);
% Drone bắt đầu ở các vị trí ngẫu nhiên xung quanh
p0 = [2, 4, 1;
      6, -2, 3;
      -1, 5, 5;
      3, 1, -2];

fprintf('=== ĐỘI HÌNH MONG MUỐN ===\n');
for i = 1:n
    fprintf('Drone %d: (%.1f, %.1f, %.1f)\n', i, p_star(i,1), p_star(i,2), p_star(i,3));
end

fprintf('\n=== VỊ TRÍ BAN ĐẦU ===\n');
for i = 1:n
    fprintf('Drone %d: (%.1f, %.1f, %.1f)\n', i, p0(i,1), p0(i,2), p0(i,3));
end

%% MÔ PHỎNG VỚI LUẬT (6.5)
% Chuyển về vector cột
p = p0(:);
p_star_vec = p_star(:);

% Lưu lịch sử
p_history = zeros(length(t), n*d);
error_history = zeros(length(t), n*d);
pos_error_norm = zeros(length(t), 1);

fprintf('\n=== BẮT ĐẦU MÔ PHỎNG ===\n');
fprintf('k_p = %.2f\n', k_p);

for k = 1:length(t)
    % Lưu lịch sử
    p_history(k,:) = p';
    
    % Tính sai số vị trí
    delta = p_star_vec - p;
    error_history(k,:) = delta';
    
    % Tính norm sai số
    pos_error_norm(k) = norm(delta);
    
    % LUẬT ĐIỀU KHIỂN (6.6): ṗ = k_p * delta - kron(L, I) * delta
    p_dot = k_p * delta + kron(L_laplacian, eye(d)) * delta;
    
    % Cập nhật vị trí
    p = p + dt * p_dot;
    
    % In tiến trình
    if mod(k, round(length(t)/5)) == 0
        fprintf('t = %.1f s, sai số = %.4f\n', t(k), pos_error_norm(k));
    end
end

%% XỬ LÝ KẾT QUẢ
% Chuyển về dạng mảng 3D [time, drone, tọa độ]
p_reshaped = reshape(p_history, [length(t), n, d]);

% Vị trí cuối
p_final = reshape(p, [n, d]);

%% VẼ KẾT QUẢ
figure('Position', [50 50 1600 900]);

% 1. Quỹ đạo 3D
subplot(2,3,1);
colors = {'r', 'b', 'g', 'm'};
markers = {'o', 's', '^', 'd'};

for i = 1:n
    x = squeeze(p_reshaped(:,i,1));
    y = squeeze(p_reshaped(:,i,2));
    z = squeeze(p_reshaped(:,i,3));
    
    plot3(x, y, z, colors{i}, 'LineWidth', 1.5); hold on
    plot3(x(1), y(1), z(1), [colors{i}, markers{i}], 'MarkerSize', 10, 'MarkerFaceColor', colors{i});
    plot3(x(end), y(end), z(end), [colors{i}, markers{i}], 'MarkerSize', 10, 'MarkerFaceColor', 'none', 'LineWidth', 2);
end
plot3(p_star(:,1), p_star(:,2), p_star(:,3), 'k*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('x'); ylabel('y'); zlabel('z');
title(sprintf('Quỹ đạo 3D (k_p = %.2f)', k_p));
legend('Drone1', 'Drone2', 'Drone3', 'Drone4', 'start', 'end', 'target', 'Location', 'best');
grid on; view(45, 30);

% 2. Hình chiếu XY
subplot(2,3,2);
for i = 1:n
    x = squeeze(p_reshaped(:,i,1));
    y = squeeze(p_reshaped(:,i,2));
    plot(x, y, colors{i}, 'LineWidth', 1.5); hold on
    plot(x(1), y(1), [colors{i}, markers{i}], 'MarkerSize', 10, 'MarkerFaceColor', colors{i});
    plot(x(end), y(end), [colors{i}, markers{i}], 'MarkerSize', 10, 'MarkerFaceColor', 'none', 'LineWidth', 2);
end
plot(p_star(:,1), p_star(:,2), 'k*', 'MarkerSize', 15);
xlabel('x'); ylabel('y');
title('Hình chiếu XY');
grid on; axis equal;

% 3. Sai số vị trí theo thời gian
subplot(2,3,3);
semilogy(t, pos_error_norm, 'b-', 'LineWidth', 2);
xlabel('t (s)'); ylabel('||δ||');
title('Sai số vị trí tổng hợp');
grid on;

% 4. Sai số từng drone
subplot(2,3,4);
drone_errors = zeros(length(t), n);
for i = 1:n
    idx = (i-1)*d + (1:d);
    drone_errors(:,i) = sqrt(sum(error_history(:,idx).^2, 2));
end
semilogy(t, drone_errors, 'LineWidth', 1.5);
xlabel('t (s)'); ylabel('||δ_i||');
title('Sai số từng drone');
legend('Drone1', 'Drone2', 'Drone3', 'Drone4', 'Location', 'best');
grid on;

% 5. Vị trí cuối trong 3D
subplot(2,3,5);
for i = 1:n
    plot3(p_final(i,1), p_final(i,2), p_final(i,3), [colors{i}, markers{i}], ...
          'MarkerSize', 20, 'MarkerFaceColor', colors{i}); hold on
    text(p_final(i,1), p_final(i,2), p_final(i,3)+0.3, sprintf('Drone%d', i), ...
         'FontSize', 12, 'FontWeight', 'bold');
end
plot3(p_star(:,1), p_star(:,2), p_star(:,3), 'k*', 'MarkerSize', 15);
for i = 1:n
    text(p_star(i,1), p_star(i,2), p_star(i,3)+0.3, sprintf('Target%d', i), ...
         'FontSize', 10, 'Color', 'k');
end
xlabel('x'); ylabel('y'); zlabel('z');
title('Vị trí cuối (màu) vs Mục tiêu (*)');
grid on; view(45, 30);

% 6. Khoảng cách giữa các drone
subplot(2,3,6);
% Tính khoảng cách giữa các cặp drone theo thời gian
dist_12 = zeros(length(t), 1);
dist_13 = zeros(length(t), 1);
dist_14 = zeros(length(t), 1);
dist_23 = zeros(length(t), 1);
dist_24 = zeros(length(t), 1);
dist_34 = zeros(length(t), 1);

for k = 1:length(t)
    p_k = reshape(p_history(k,:), [n, d]);
    dist_12(k) = norm(p_k(1,:) - p_k(2,:));
    dist_13(k) = norm(p_k(1,:) - p_k(3,:));
    dist_14(k) = norm(p_k(1,:) - p_k(4,:));
    dist_23(k) = norm(p_k(2,:) - p_k(3,:));
    dist_24(k) = norm(p_k(2,:) - p_k(4,:));
    dist_34(k) = norm(p_k(3,:) - p_k(4,:));
end

plot(t, dist_12, 'r-', 'LineWidth', 1); hold on
plot(t, dist_13, 'b-', 'LineWidth', 1);
plot(t, dist_14, 'g-', 'LineWidth', 1);
plot(t, dist_23, 'm-', 'LineWidth', 1);
plot(t, dist_24, 'c-', 'LineWidth', 1);
plot(t, dist_34, 'k-', 'LineWidth', 1);

% Khoảng cách mong muốn
d12_star = norm(p_star(1,:) - p_star(2,:));
d13_star = norm(p_star(1,:) - p_star(3,:));
d14_star = norm(p_star(1,:) - p_star(4,:));
d23_star = norm(p_star(2,:) - p_star(3,:));
d24_star = norm(p_star(2,:) - p_star(4,:));
d34_star = norm(p_star(3,:) - p_star(4,:));

plot(t, d12_star * ones(size(t)), 'r--', 'LineWidth', 1);
plot(t, d13_star * ones(size(t)), 'b--', 'LineWidth', 1);
plot(t, d14_star * ones(size(t)), 'g--', 'LineWidth', 1);
plot(t, d23_star * ones(size(t)), 'm--', 'LineWidth', 1);
plot(t, d24_star * ones(size(t)), 'c--', 'LineWidth', 1);
plot(t, d34_star * ones(size(t)), 'k--', 'LineWidth', 1);

xlabel('t (s)'); ylabel('Khoảng cách');
title('Khoảng cách giữa các drone (nét đứt: mong muốn)');
legend('1-2', '1-3', '1-4', '2-3', '2-4', '3-4', 'Location', 'best');
grid on;

%% TÍNH TOÁN KẾT QUẢ CUỐI
fprintf('\n=== KẾT QUẢ CUỐI CÙNG ===\n');
fprintf('t = %.1f s\n', T);
fprintf('Sai số vị trí cuối: %.6f\n', pos_error_norm(end));

fprintf('\nVị trí cuối của các drone:\n');
for i = 1:n
    error_i = norm(p_final(i,:) - p_star(i,:));
    fprintf('Drone %d: (%.3f, %.3f, %.3f) - Sai số: %.4f\n', ...
            i, p_final(i,1), p_final(i,2), p_final(i,3), error_i);
end

fprintf('\nKhoảng cách cuối giữa các cặp:\n');
fprintf('Cặp 1-2: %.3f (mong muốn: %.3f) - Sai số: %.4f\n', ...
        dist_12(end), d12_star, abs(dist_12(end)-d12_star));
fprintf('Cặp 1-3: %.3f (mong muốn: %.3f) - Sai số: %.4f\n', ...
        dist_13(end), d13_star, abs(dist_13(end)-d13_star));
fprintf('Cặp 1-4: %.3f (mong muốn: %.3f) - Sai số: %.4f\n', ...
        dist_14(end), d14_star, abs(dist_14(end)-d14_star));
fprintf('Cặp 2-3: %.3f (mong muốn: %.3f) - Sai số: %.4f\n', ...
        dist_23(end), d23_star, abs(dist_23(end)-d23_star));
fprintf('Cặp 2-4: %.3f (mong muốn: %.3f) - Sai số: %.4f\n', ...
        dist_24(end), d24_star, abs(dist_24(end)-d24_star));
fprintf('Cặp 3-4: %.3f (mong muốn: %.3f) - Sai số: %.4f\n', ...
        dist_34(end), d34_star, abs(dist_34(end)-d34_star));

%% ANIMATION 3D
figure('Position', [100 100 1200 800]);

for k = 1:20:length(t)  % chỉ hiển thị 1/20 số frame
    clf;
    
    % Vẽ drone tại thời điểm hiện tại
    p_current = reshape(p_history(k,:), [n, d]);
    
    for i = 1:n
        plot3(p_current(i,1), p_current(i,2), p_current(i,3), ...
              [colors{i}, markers{i}], 'MarkerSize', 20, 'MarkerFaceColor', colors{i}); hold on
        text(p_current(i,1), p_current(i,2), p_current(i,3)+0.5, ...
             sprintf('%d', i), 'FontSize', 14, 'FontWeight', 'bold');
    end
    
    % Vẽ các cạnh kết nối
    edges_list = [1,2; 1,3; 1,4; 2,3; 2,4; 3,4];
    for e = 1:size(edges_list,1)
        i = edges_list(e,1);
        j = edges_list(e,2);
        plot3([p_current(i,1), p_current(j,1)], ...
              [p_current(i,2), p_current(j,2)], ...
              [p_current(i,3), p_current(j,3)], 'k--', 'LineWidth', 1);
    end
    
    % Vẽ vị trí đích
    plot3(p_star(:,1), p_star(:,2), p_star(:,3), 'k*', 'MarkerSize', 15);
    
    xlabel('x'); ylabel('y'); zlabel('z');
    title(sprintf('t = %.2f s, Sai số = %.4f', t(k), pos_error_norm(k)));
    xlim([-5, 10]); ylim([-5, 10]); zlim([-5, 10]);
    grid on; view(45, 30);
    
    drawnow;
    pause(0.01);
end