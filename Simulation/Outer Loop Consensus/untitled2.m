%% MÔ PHỎNG 5 DRONE - ĐỒ THỊ CỨNG 9 CẠNH
clear; clc; close all;

%% ========================================================================
%  1. THAM SỐ
%% ========================================================================
n_drones = 5;
dim = 3;
dt = 0.02;
T = 60;
t = 0:dt:T;
n_steps = length(t);

% Tham số điều khiển
alpha = 1;
beta_leader = 100;
beta = 50.0;
gamma_leader = 50.0;

% Quỹ đạo hình tròn cho leader
R = 15;
h = 8;
vz = 0.5;
omega = 0.2;

%% ========================================================================
%  2. ĐỘI HÌNH MONG MUỐN
%% ========================================================================
% Drone 1 (leader) ở đỉnh, 4 followers ở đáy hình vuông
L = 6;       % khoảng cách từ tâm đáy đến các góc
H = 8;       % chiều cao của hình chóp

% Vị trí mong muốn so với leader
p_rel_star = zeros(dim, n_drones);
p_rel_star(:,1) = [0; 0; 0];           % leader ở đỉnh
p_rel_star(:,2) = [L; L; -H];          % follower 2
p_rel_star(:,3) = [L; -L; -H];         % follower 3
p_rel_star(:,4) = [-L; L; -H];         % follower 4
p_rel_star(:,5) = [-L; -L; -H];        % follower 5

%% ========================================================================
%  3. ĐỒ THỊ CỨNG 9 CẠNH (3n-6 = 9)
%% ========================================================================
edges = [1 2; 1 3; 1 4; 1 5;   % leader với all followers (4 cạnh)
         2 3; 2 4; 3 4;         % tam giác giữa 2,3,4 (3 cạnh)
         3 5; 4 5];             % nối 5 với 3 và 4 (2 cạnh)

m = size(edges, 1);
fprintf('Số cạnh: %d (yêu cầu tối thiểu: %d)\n', m, 3*n_drones-6);

% Kiểm tra
if m < 3*n_drones-6
    warning('Đồ thị không đủ cứng! Cần %d cạnh, chỉ có %d', 3*n_drones-6, m);
else
    fprintf('Đồ thị đủ cứng trong 3D\n');
end

% Danh sách neighbors
neighbors = cell(n_drones, 1);
for i = 1:n_drones
    neighbors{i} = [];
    for e = 1:m
        if edges(e,1) == i
            neighbors{i} = [neighbors{i}, edges(e,2)];
        elseif edges(e,2) == i
            neighbors{i} = [neighbors{i}, edges(e,1)];
        end
    end
    neighbors{i} = sort(unique(neighbors{i}));
    fprintf('Drone %d neighbors: ', i);
    fprintf('%d ', neighbors{i});
    fprintf('\n');
end

% Tính khoảng cách mong muốn
d_star = zeros(m, 1);
fprintf('\n=== KHOẢNG CÁCH MONG MUỐN ===\n');
for e = 1:m
    i = edges(e,1); j = edges(e,2);
    d_star(e) = norm(p_rel_star(:,i) - p_rel_star(:,j));
    fprintf('  d%d%d* = %.2f m\n', i, j, d_star(e));
end

%% ========================================================================
%  4. KHỞI TẠO
%% ========================================================================
P = zeros(dim, n_drones, n_steps);
V = zeros(dim, n_drones, n_steps);

% Leader bắt đầu tại gốc
P(:,1,1) = [0; 0; h];

% Followers bắt đầu lệch
for i = 2:n_drones
    P(:,i,1) = P(:,1,1) + p_rel_star(:,i) + 3*randn(3,1);
end
V(:,:,1) = zeros(dim, n_drones);

%% ========================================================================
%  5. VÒNG LẶP MÔ PHỎNG
%% ========================================================================
fprintf('\n=== BẮT ĐẦU MÔ PHỎNG ===\n');

formation_error = zeros(n_steps, 1);
leader_error = zeros(n_steps, 1);

for k = 1:n_steps-1
    p_curr = P(:,:,k);
    v_curr = V(:,:,k);
    
    % =====================================================================
    %  LEADER (Drone 1)
    % =====================================================================
    i = 1;
    p_ref = [R*cos(omega*t(k)); R*sin(omega*t(k)); vz*t(k)];
    v_ref = [-R*omega*sin(omega*t(k)); R*omega*cos(omega*t(k)); 0];
    u_leader = -gamma_leader * (p_curr(:,i) - p_ref) - beta_leader * (v_curr(:,i) - v_ref);
    u(:,i) = u_leader;
    leader_error(k+1) = norm(p_curr(:,i) - p_ref);
    
    % =====================================================================
    %  FOLLOWERS (Drone 2-5)
    % =====================================================================
    for i = 2:n_drones
        u_dist = zeros(dim, 1);
        
        for j = neighbors{i}
            p_ij = p_curr(:,j) - p_curr(:,i);
            dij = norm(p_ij);
            
            if dij > 1e-6
                % Tìm chỉ số cạnh
                e_idx = 0;
                for e = 1:m
                    if (edges(e,1) == i && edges(e,2) == j) || ...
                       (edges(e,1) == j && edges(e,2) == i)
                        e_idx = e;
                        break;
                    end
                end
                
                if e_idx > 0
                    u_dist = u_dist + alpha*(dij^2 - d_star(e_idx)^2) * p_ij;
                end
            end
        end
        
        u_damp = -beta * v_curr(:,i);
        u(:,i) = u_dist + u_damp;
    end
    
    % Cập nhật
    V(:,:,k+1) = V(:,:,k) + u * dt;
    P(:,:,k+1) = P(:,:,k) + V(:,:,k) * dt;
    
    % Tính sai số đội hình (trên tất cả các cạnh)
    error_sum = 0;
    for e = 1:m
        i = edges(e,1); j = edges(e,2);
        dij = norm(P(:,i,k+1) - P(:,j,k+1));
        error_sum = error_sum + (dij - d_star(e))^2;
    end
    formation_error(k+1) = sqrt(error_sum / m);
    
    if mod(k, round(n_steps/10)) == 0
        fprintf('  t = %.1f s, leader error = %.3f m, form error = %.3f m\n', ...
            k*dt, leader_error(k), formation_error(k));
    end
end

fprintf('Mô phỏng hoàn tất!\n');
fprintf('Sai số cuối: leader = %.4f m, formation = %.4f m\n', ...
    leader_error(end), formation_error(end));

%% ========================================================================
%  6. VẼ KẾT QUẢ
%% ========================================================================
figure('Name', '5 Drone - Đồ thị cứng 9 cạnh', 'Position', [50, 50, 1400, 900]);

%% 6.1. Quỹ đạo 3D
subplot(2,3,[1,4]);
colors = lines(n_drones);
hold on; grid on; box on;

% Quỹ đạo tham chiếu của leader (hình tròn)
theta = 0:0.1:2*pi;
x_ref = R*cos(theta);
y_ref = R*sin(theta);
z_ref = h*ones(size(theta));
plot3(x_ref, y_ref, z_ref, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Quỹ đạo leader');

% Quỹ đạo thực tế
for i = 1:n_drones
    if i == 1
        plot3(squeeze(P(1,i,:)), squeeze(P(2,i,:)), squeeze(P(3,i,:)), ...
            'Color', 'r', 'LineWidth', 3, 'DisplayName', 'Leader');
    else
        plot3(squeeze(P(1,i,:)), squeeze(P(2,i,:)), squeeze(P(3,i,:)), ...
            'Color', colors(i,:), 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Follower %d', i));
    end
end

% Điểm đầu
for i = 1:n_drones
    plot3(P(1,i,1), P(2,i,1), P(3,i,1), 'o', 'Color', colors(i,:), ...
        'MarkerSize', 8, 'MarkerFaceColor', 'w');
end

% Điểm cuối
for i = 1:n_drones
    plot3(P(1,i,end), P(2,i,end), P(3,i,end), 's', 'Color', colors(i,:), ...
        'MarkerSize', 10, 'MarkerFaceColor', colors(i,:));
end

xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('5 drone - Đồ thị cứng 9 cạnh');
legend('Location', 'best');
view(45, 30); axis equal;

%% 6.2. Sai số leader
subplot(2,3,2);
plot(t, leader_error, 'r-', 'LineWidth', 2);
xlabel('Thời gian (s)'); ylabel('Sai số (m)');
title('Sai số bám quỹ đạo leader');
grid on;

%% 6.3. Sai số đội hình
subplot(2,3,3);
plot(t, formation_error, 'b-', 'LineWidth', 2);
xlabel('Thời gian (s)'); ylabel('Sai số RMS (m)');
title('Sai số đội hình (9 cạnh)');
grid on;

%% 6.4. Khoảng cách
subplot(2,3,5);
hold on; grid on; box on;

for e = 1:m
    i = edges(e,1); j = edges(e,2);
    dist = squeeze(sqrt(sum((P(:,i,:) - P(:,j,:)).^2, 1)));
    plot(t, dist(:), 'LineWidth', 1, 'DisplayName', sprintf('d_{%d%d}', i, j));
end

% Đường mong muốn
for e = 1:m
    yline(d_star(e), '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.5);
end

xlabel('Thời gian (s)'); ylabel('Khoảng cách (m)');
title('Khoảng cách giữa các drone');
legend('Location', 'best', 'NumColumns', 2);

%% 6.5. Animation
subplot(2,3,6);
for k = 1:100:n_steps
    cla; hold on; grid on; box on;
    
    p_curr = P(:,:,k);
    
    % Vẽ leader
    plot3(p_curr(1,1), p_curr(2,1), p_curr(3,1), 'ro', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'r');
    
    % Vẽ followers
    for i = 2:n_drones
        plot3(p_curr(1,i), p_curr(2,i), p_curr(3,i), 'o', ...
            'Color', colors(i,:), 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
    end
    
    % Vẽ các cạnh
    for e = 1:m
        i = edges(e,1); j = edges(e,2);
        plot3([p_curr(1,i) p_curr(1,j)], [p_curr(2,i) p_curr(2,j)], ...
              [p_curr(3,i) p_curr(3,j)], 'g-', 'LineWidth', 1);
    end
    
    % Vẽ quỹ đạo tham chiếu
    theta = 0:0.1:2*pi;
    plot3(R*cos(theta), R*sin(theta), h*ones(size(theta)), 'k--');
    
    xlabel('x'); ylabel('y'); zlabel('z');
    title(sprintf('t = %.1f s', t(k)));
    xlim([-R-5, R+5]); ylim([-R-5, R+5]); zlim([0, 2*h]);
    view(45, 30);
    
    drawnow;
    pause(0.01);
end

sgtitle(sprintf('5 drone - %d cạnh (tối thiểu %d)', m, 3*n_drones-6), ...
    'FontSize', 14, 'FontWeight', 'bold');