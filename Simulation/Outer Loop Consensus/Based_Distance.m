%% MÔ PHỎNG ĐIỀU KHIỂN ĐỘI HÌNH DỰA TRÊN KHOẢNG CÁCH
% 4 drone trong mặt phẳng 2D (dễ vẽ)
clear; clc; close all;

%% ========================================================================
%  1. THAM SỐ
%% ========================================================================
n_drones = 4;           % số drone
dim = 2;                 % không gian 2D (dễ vẽ)
dt = 0.01;               % bước thời gian
T = 30;                  % tổng thời gian
t = 0:dt:T;              % vector thời gian
n_steps = length(t);

%% ========================================================================
%  2. ĐỘI HÌNH MONG MUỐN - HÌNH VUÔNG CẠNH 5
%% ========================================================================
L = 20;  % cạnh hình vuông

% Vị trí mong muốn (tâm tại gốc)
p_star = zeros(dim, n_drones);
p_star(:,1) = [-L/2; -L/2];  % drone 1
p_star(:,2) = [ L/2; -L/2];  % drone 2
p_star(:,3) = [ L/2;  L/2];  % drone 3
p_star(:,4) = [-L/2;  L/2];  % drone 4

%% ========================================================================
%  3. ĐỒ THỊ VÀ KHOẢNG CÁCH MONG MUỐN
%% ========================================================================
% Đồ thị: hình vuông đủ 4 cạnh + 1 đường chéo (để cứng)
edges = [1 2; 2 3; 3 4; 4 1; 1 3];  % 5 cạnh
m = size(edges, 1);

% Tính khoảng cách mong muốn từ p_star
d_star = zeros(m, 1);
for e = 1:m
    i = edges(e,1);
    j = edges(e,2);
    d_star(e) = norm(p_star(:,i) - p_star(:,j));
end

% Hiển thị khoảng cách
fprintf('Khoảng cách mong muốn:\n');
for e = 1:m
    fprintf('  d%d%d = %.2f\n', edges(e,1), edges(e,2), d_star(e));
end

%% ========================================================================
%  4. KHỞI TẠO VỊ TRÍ BAN ĐẦU (LỆCH SO VỚI MONG MUỐN)
%% ========================================================================
P = zeros(dim, n_drones, n_steps);
P(:,:,1) = p_star + 3 * randn(dim, n_drones);  % nhiễu ngẫu nhiên

% Vẽ vị trí ban đầu
fprintf('\nVị trí ban đầu:\n');
disp(P(:,:,1)');

%% ========================================================================
%  5. VÒNG LẶP MÔ PHỎNG
%% ========================================================================
fprintf('\nĐang mô phỏng...\n');

formation_error = zeros(n_steps, 1);

for k = 1:n_steps-1
    p_curr = P(:,:,k);
    
    % Tính lực điều khiển cho từng drone
    u = zeros(dim, n_drones);
    
    for i = 1:n_drones
        for e = 1:m
            if edges(e,1) == i
                j = edges(e,2);
                % Tính khoảng cách hiện tại
                dij = norm(p_curr(:,i) - p_curr(:,j));
                % Lực điều khiển
                u(:,i) = u(:,i) + (dij^2 - d_star(e)^2) * (p_curr(:,j) - p_curr(:,i));
                
            elseif edges(e,2) == i
                j = edges(e,1);
                dij = norm(p_curr(:,i) - p_curr(:,j));
                u(:,i) = u(:,i) + (dij^2 - d_star(e)^2) * (p_curr(:,j) - p_curr(:,i));
            end
        end
    end
    
    % Giới hạn lực (tránh sốc)
    max_u = 5;
    u = max(min(u, max_u), -max_u);
    
    % Cập nhật vị trí (Euler)
    P(:,:,k+1) = P(:,:,k) + u * dt;
    
    % Tính sai số đội hình (RMS)
    error_sum = 0;
    count = 0;
    for e = 1:m
        i = edges(e,1); j = edges(e,2);
        dij = norm(P(:,i,k+1) - P(:,j,k+1));
        error_sum = error_sum + (dij - d_star(e))^2;
        count = count + 1;
    end
    formation_error(k+1) = sqrt(error_sum / count);
    
    % Hiển thị tiến độ
    if mod(k, round(n_steps/10)) == 0
        fprintf('  t = %.1f s, sai số = %.4f\n', k*dt, formation_error(k));
    end
end

fprintf('Mô phỏng hoàn tất!\n');

%% ========================================================================
%  6. VẼ KẾT QUẢ
%% ========================================================================
figure('Name', 'Kết quả mô phỏng', 'Position', [100, 100, 1400, 900]);

%% 6.1. Quỹ đạo
subplot(2,3,[1,4]);
colors = lines(n_drones);
hold on; grid on; box on;

for i = 1:n_drones
    plot(squeeze(P(1,i,:)), squeeze(P(2,i,:)), ...
        'Color', colors(i,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Drone %d', i));
    
    % Điểm đầu
    plot(P(1,i,1), P(2,i,1), 'o', 'Color', colors(i,:), ...
        'MarkerSize', 8, 'MarkerFaceColor', 'w');
    
    % Điểm cuối
    plot(P(1,i,end), P(2,i,end), 's', 'Color', colors(i,:), ...
        'MarkerSize', 10, 'MarkerFaceColor', colors(i,:));
end

% Vẽ đội hình mong muốn
plot(p_star(1,:), p_star(2,:), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'none');
for e = 1:m
    i = edges(e,1); j = edges(e,2);
    plot([p_star(1,i) p_star(1,j)], [p_star(2,i) p_star(2,j)], ...
        'k--', 'LineWidth', 1);
end

xlabel('x (m)'); ylabel('y (m)');
title('Quỹ đạo drone');
legend('Location', 'best');
axis equal;

%% 6.2. Sai số đội hình
subplot(2,3,2);
plot(t, formation_error, 'b-', 'LineWidth', 2);
xlabel('Thời gian (s)'); ylabel('Sai số RMS (m)');
title('Sai số đội hình');
grid on;

%% 6.3. Khoảng cách theo thời gian
subplot(2,3,3);
hold on; grid on; box on;

for e = 1:m
    i = edges(e,1); j = edges(e,2);
    dist = squeeze(sqrt(sum((P(:,i,:) - P(:,j,:)).^2, 1)));
    plot(t, dist(:), 'LineWidth', 1.5, 'DisplayName', sprintf('d_{%d%d}', i, j));
end

% Vẽ đường mong muốn
for e = 1:m
    yline(d_star(e), '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
end

xlabel('Thời gian (s)'); ylabel('Khoảng cách (m)');
title('Khoảng cách giữa các drone');
legend('Location', 'best');

%% 6.4. Animation
subplot(2,3,[5,6]);
for k = 1:100:n_steps
    cla;
    hold on; grid on; box on;
    
    % Vị trí hiện tại
    p_curr = P(:,:,k);
    
    for i = 1:n_drones
        plot(p_curr(1,i), p_curr(2,i), 'o', 'Color', colors(i,:), ...
            'MarkerSize', 10, 'MarkerFaceColor', colors(i,:));
        text(p_curr(1,i)+0.3, p_curr(2,i)+0.3, num2str(i), ...
            'Color', colors(i,:), 'FontWeight', 'bold');
    end
    
    % Vẽ các cạnh hiện tại
    for e = 1:m
        i = edges(e,1); j = edges(e,2);
        plot([p_curr(1,i) p_curr(1,j)], [p_curr(2,i) p_curr(2,j)], ...
            'g-', 'LineWidth', 1);
    end
    
    % Vẽ đội hình mong muốn (mờ)
    plot(p_star(1,:), p_star(2,:), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'none');
    
    xlabel('x (m)'); ylabel('y (m)');
    title(sprintf('t = %.1f s, sai số = %.3f m', t(k), formation_error(k)));
    xlim([-15, 15]); ylim([-15, 15]);
    axis equal;
    
    drawnow;
    pause(0.01);
end

sgtitle('Điều khiển đội hình dựa trên khoảng cách', 'FontSize', 14, 'FontWeight', 'bold');