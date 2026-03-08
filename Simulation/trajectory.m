clc; clear; close all;

% Thời gian
t = linspace(0, 20, 1000);   % 0 -> 20s, 1000 điểm
theta = 2*pi*t/10;           % góc quay quanh z (1 vòng trong 10s)

% Quỹ đạo của vật O (tâm)
R = 10;                      % bán kính quỹ đạo tròn
z_speed = 0.5;               % tốc độ tăng theo z
O = [R*cos(theta); R*sin(theta); z_speed*t];  % (x,y,z)

% Vị trí 4 vật trong hệ cục bộ (hình vuông cạnh 10, tâm O)
a = 5;
local_pos = [ -a, -a, 0;
               a, -a, 0;
               a,  a, 0;
              -a,  a, 0 ]';

% Quỹ đạo toàn cục của 4 vật
traj = cell(4,1);
for k = 1:4
    traj{k} = zeros(3, length(t));
    for i = 1:length(t)
        Rz = [cos(theta(i)) -sin(theta(i)) 0;
              sin(theta(i))  cos(theta(i)) 0;
              0              0             1];
        traj{k}(:,i) = O(:,i) + Rz*local_pos(:,k);
    end
end

% Vẽ quỹ đạo 3D
figure; hold on; grid on; axis equal;
plot3(O(1,:), O(2,:), O(3,:), 'k', 'LineWidth', 2); % quỹ đạo vật O
colors = ['r','g','b','m'];
for k = 1:4
    plot3(traj{k}(1,:), traj{k}(2,:), traj{k}(3,:), colors(k), 'LineWidth', 1.5);
end
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Quỹ đạo 5 vật trong không gian 3D');
legend('O (tâm)', 'Vật 1','Vật 2','Vật 3','Vật 4');
view(3); % góc nhìn 3D