%% Mô phỏng leader-follower: 4 drone bay theo quỹ đạo tròn
clear; clc; close all;

%% Tham số
dt = 0.01;
T = 60;
t = 0:dt:T;
n = 4;
dim = 3;

L = 5;  % khoảng cách giữa các drone

%% Đội hình mong muốn - Tứ diện
p_star = zeros(dim, n);
p_star(:,1) = [0; 0; 0];           % leader
p_star(:,2) = [L; 0; 0];           % follower 1
p_star(:,3) = [L/2; L*sqrt(3)/2; 0]; % follower 2
p_star(:,4) = [L/2; L*sqrt(3)/6; L*sqrt(2/3)]; % follower 3

%% Đồ thị và khoảng cách mong muốn
edges = [1 2; 1 3; 1 4; 2 3; 2 4; 3 4];
m = size(edges, 1);

d_star = zeros(m, 1);
for k = 1:m
    i = edges(k,1); j = edges(k,2);
    d_star(k) = norm(p_star(:,i) - p_star(:,j));
end

%% Khởi tạo vị trí
P = zeros(dim, n, length(t));
% Leader bắt đầu tại gốc, followers lệch một chút
P(:,:,1) = p_star + [zeros(3,1), 2*randn(3,3)];

%% Vòng lặp mô phỏng
for k = 1:length(t)-1
    p_curr = P(:,:,k);
    
    % Quỹ đạo mong muốn cho leader (bay vòng tròn + lên cao dần)
    R = 10;  % bán kính
    omega = 0.2;  % tốc độ góc
    p_leader_desired = [R*cos(omega*t(k)); R*sin(omega*t(k)); 2 + 0.05*t(k)];
    
    % Tính lực điều khiển
    u = zeros(dim, n);
    
    % Leader: vừa giữ đội hình, vừa bám quỹ đạo
    i = 1;  % leader
    u_formation = zeros(3,1);
    for e = 1:m
        if edges(e,1) == i
            j = edges(e,2);
            dij = norm(p_curr(:,i) - p_curr(:,j));
            u_formation = u_formation - (dij^2 - d_star(e)^2) * (p_curr(:,i) - p_curr(:,j));
        elseif edges(e,2) == i
            j = edges(e,1);
            dij = norm(p_curr(:,i) - p_curr(:,j));
            u_formation = u_formation - (dij^2 - d_star(e)^2) * (p_curr(:,i) - p_curr(:,j));
        end
    end
    u_tracking = 2 * (p_leader_desired - p_curr(:,1));  % kéo leader về quỹ đạo
    u(:,1) = u_formation + u_tracking;
    
    % Followers: chỉ giữ đội hình với neighbors
    for i = 2:n
        for e = 1:m
            if edges(e,1) == i
                j = edges(e,2);
                dij = norm(p_curr(:,i) - p_curr(:,j));
                u(:,i) = u(:,i) - (dij^2 - d_star(e)^2) * (p_curr(:,i) - p_curr(:,j));
            elseif edges(e,2) == i
                j = edges(e,1);
                dij = norm(p_curr(:,i) - p_curr(:,j));
                u(:,i) = u(:,i) - (dij^2 - d_star(e)^2) * (p_curr(:,i) - p_curr(:,j));
            end
        end
    end
    
    % Cập nhật vị trí
    P(:,:,k+1) = P(:,:,k) + u * dt;
end

%% Vẽ kết quả
figure(1); clf; hold on;
colors = lines(n);

% Vẽ quỹ đạo của leader
plot3(squeeze(P(1,1,:)), squeeze(P(2,1,:)), squeeze(P(3,1,:)), ...
    'r-', 'LineWidth', 3, 'DisplayName', 'Leader trajectory');

% Vẽ quỹ đạo của followers
for i = 2:n
    plot3(squeeze(P(1,i,:)), squeeze(P(2,i,:)), squeeze(P(3,i,:)), ...
        'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', ['Follower ' num2str(i)]);
end

% Vẽ vị trí cuối
for i = 1:n
    plot3(P(1,i,end), P(2,i,end), P(3,i,end), 'o', ...
        'Color', colors(i,:), 'MarkerSize', 10, 'MarkerFaceColor', colors(i,:));
end

xlabel('x'); ylabel('y'); zlabel('z');
title('Leader-Follower: 4 drone bay theo quỹ đạo tròn');
legend('Location', 'best');
grid on; axis equal;
view(45, 30);