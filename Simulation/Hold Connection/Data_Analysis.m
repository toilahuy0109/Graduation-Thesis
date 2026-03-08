% =============================================================
%                          DATA ANALYSIS
% =============================================================

%% Parameters
n_drones = 5;          % Number of drones

my_logsout = out.logsout;

%% Get data from simulink
p_ref = my_logsout.get('p_ref');
time = p_ref.Values.Time;
p_ref = p_ref.Values.Data;

pos1 = my_logsout.get('pos1');
pos1_data = pos1.Values.Data;

pos2 = my_logsout.get('pos2');
pos2_data = pos2.Values.Data;

pos3 = my_logsout.get('pos3');
pos3_data = pos3.Values.Data;

pos4 = my_logsout.get('pos4');
pos4_data = pos4.Values.Data;

pos5 = my_logsout.get('pos5');
pos5_data = pos5.Values.Data;

pos = {pos1_data, pos2_data, pos3_data, pos4_data, pos5_data};
pos_ref = {p_ref(1:3,:,:), p_ref(4:6,:,:), p_ref(7:9,:,:), p_ref(10:12,:,:), p_ref(13:15,:,:)};

%% Data Memories
% Position Error
ex = zeros(5,length(time));
ey = zeros(5,length(time));
ez = zeros(5,length(time));

for i = 1:length(time)
    for j = 1:n_drones
        ex(j,i) = abs(pos{j}(1,:,i) - pos_ref{j}(1,:,i));
        ey(j,i) = abs(pos{j}(2,:,i) - pos_ref{j}(2,:,i));
        ez(j,i) = abs(pos{j}(3,:,i) - pos_ref{j}(3,:,i));
    end
end

%% Plot
% Configuration
colors = lines(n_drones);
line_styles = {'-', '--', ':', '-.', '-'};

% Figure 1
figure('Name', 'Tracking Position of Drone');

% Error X
subplot(2,3,1);
hold on; grid on; box on;

for i = 1:n_drones
    plot(time, ex(i,:), 'Color', colors(i,:), 'LineWidth', 1.5);
end
xlabel('Time (s)', 'FontSize', 11);
ylabel('Error (m)', 'FontSize', 12);
title('Error X of 5 drones', 'FontSize', 12);
legend(arrayfun(@(x) sprintf('Drone %d', x), 1:n_drones, 'UniformOutput',false));

% Error Y
subplot(2,3,2);
hold on; grid on; box on;

for i = 1:n_drones
    plot(time, ey(i,:), 'Color', colors(i,:), 'LineWidth', 1.5);
end
xlabel('Time (s)', 'FontSize', 11);
ylabel('Error (m)', 'FontSize', 12);
title('Error Y of 5 drones', 'FontSize', 12);
legend(arrayfun(@(x) sprintf('Drone %d', x), 1:n_drones, 'UniformOutput',false));

% Error Z
subplot(2,3,3);
hold on; grid on; box on;

for i = 1:n_drones
    plot(time, ez(i,:), 'Color', colors(i,:), 'LineWidth', 1.5);
end
xlabel('Time (s)', 'FontSize', 11);
ylabel('Error (m)', 'FontSize', 12);
title('Error Z of 5 drones', 'FontSize', 12);
legend(arrayfun(@(x) sprintf('Drone %d', x), 1:n_drones, 'UniformOutput',false));


% Tracking Reference Trajectory
subplot(2,3,4);
hold on; grid on; box on;

for i = 1:n_drones
    plot(time, pos_ref{i}(1,:,:), 'Color', colors(i,:));
end