% =============================================================
%                          DATA ANALYSIS
% =============================================================

close all;

%% Parameters
n_drones = 10;          % Number of drones

my_logsout = out.logsout;

%% Get data from simulink
p_ref = my_logsout.get('p_ref');
time = p_ref.Values.Time;
p_ref = p_ref.Values.Data;

p_data = my_logsout.get('p');
p = p_data.Values.Data;

v_data = my_logsout.get('v');
v = v_data.Values.Data;

pes_data = my_logsout.get('p_es');
p_es = pes_data.Values.Data;


n_edges = size(edges,1);

pos = cell(n_drones);
pes = cell(n_drones);

for i = 1:n_drones
    pos{i} = p(3*(i-1)+1 : 3*i, :, :);
    pes{i} = p_es(3*(i-1)+1 : 3*i, :, :);
end

%% Data Memories
% Position Error
ex = zeros(1,length(time));
ey = zeros(1,length(time));
ez = zeros(1,length(time));

for i = 1:length(time)
    ex(1,i) = norm(p_ref(1,:,i) - pos{1}(1,:,i));
    ey(1,i) = norm(p_ref(2,:,i) - pos{1}(2,:,i));
    ez(1,i) = norm(p_ref(3,:,i) - pos{1}(3,:,i));
end

%% Plot
% Configuration
colors = lines(n_edges);
line_styles = {'-', '--', ':', '-.', '-'};

% Figure 1
figure1 = figure('Name', 'Tracking Position of Drone');

% Error X
subplot(2,3,1);
hold on; grid on; box on;

plot(time, ex(1,:), 'Color', colors(1,:), 'LineWidth', 1.5);

xlabel('Time (s)', 'FontSize', 11);
ylabel('Error (m)', 'FontSize', 12);
title('Error X of drone 1', 'FontSize', 12);

% Error Y
subplot(2,3,2);
hold on; grid on; box on;

plot(time, ey(1,:), 'Color', colors(1,:), 'LineWidth', 1.5);

xlabel('Time (s)', 'FontSize', 11);
ylabel('Error (m)', 'FontSize', 12);
title('Error Y of drone 1', 'FontSize', 12);

% Error Z
subplot(2,3,3);
hold on; grid on; box on;

plot(time, ez(1,:), 'Color', colors(1,:), 'LineWidth', 1.5);

xlabel('Time (s)', 'FontSize', 11);
ylabel('Error (m)', 'FontSize', 12);
title('Error Z of drone 1', 'FontSize', 12);

% Distance
dist = zeros(n_edges,length(time));

for t = 1:length(time)
    for e = 1:n_edges
        i = edges(e,1);
        j = edges(e,2);
        dist(e,t) = norm(pos{i}(:,:,t) - pos{j}(:,:,t));
    end
end

subplot(2,3,4);
hold on; grid on; box on;
for i = 1:n_edges
    plot(time, dist(i,:), 'Color', colors(i,:), 'LineWidth', 1.5);
end
xlabel("Time (s)", 'FontSize', 11);
ylabel("Distance (m)", 'FontSize', 12);
title("Distance between each agent", 'FontSize', 12);

legend(arrayfun(@(k) sprintf('Edge (%d-%d)', edges(k,1), edges(k,2)), 1:n_edges, 'UniformOutput',false));

% Error distance
err_dist = zeros(n_edges, length(time));

for t = 1:length(time)
    for e = 1:n_edges
        err_dist(e,t) = abs(dist(e,t) - d_star(e));
    end
end

subplot(2,3,5);
hold on; grid on; box on;
for i = 1:n_edges
    plot(time, err_dist(i,:), 'Color', colors(i,:), 'LineWidth', 1.5);
end
xlabel("Time (s)", 'FontSize', 11);
ylabel("Distance Error", 'FontSize', 12);
title("Distance Error of each agents", 'FontSize', 12);

% Min Distance
min_dist = zeros(1,length(time));

for t = 1:length(time)
    min_dist(t) = min(dist(:,t));
end

subplot(2,3,6);
hold on; grid on; box on;

plot(time, min_dist(1,:), 'Color', colors(1,:), 'LineWidth', 1.5);
yline(5, '--r', 'Safe Distance');

xlabel("Time (s)", 'FontSize', 11);
ylabel("Distance (m)", 'FontSize', 12);
title("Min Distance", 'FontSize', 12);

%% Data Hold Connection
lambda_data = my_logsout.get('lambda');
lambda = lambda_data.Values.Data;

min_edge = zeros(5,length(time));

for t = 1:length(time)
    for n = 1:n_drones
        for e = 1:n_edges
            i = edges(e,1);
            j = edges(e,2);
            if n == i || n == j
                if min_edge(n,t) == 0
                    min_edge(n,t) = dist(e,t);
                else
                    min_edge(n,t) = min(min_edge(n,t), dist(e,t));
                end
            else
                continue;
            end
        end
    end
end

max_edge = zeros(5, length(time));

for t = 1:length(time)
    for n = 1:n_drones
        for e = 1:n_edges
            i = edges(e,1);
            j = edges(e,2);
            if n == i || n == j
                if max_edge(n,t) == 0
                    max_edge(n,t) = dist(e,t);
                else
                    max_edge(n,t) = max(max_edge(n,t), dist(e,t));
                end
            else
                continue;
            end
        end
    end
end

figure2 = figure('Name', 'Hold Connection and Avoid Obstacle');



%% Data Avoiding Obstacle
Phi_data = my_logsout.get("Phi");
Phi = Phi_data.Values.Data;


