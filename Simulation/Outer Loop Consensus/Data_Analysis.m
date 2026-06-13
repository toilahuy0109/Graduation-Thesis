%% ============================================
%  POST-PROCESSING FROM SIMULINK DATASET
%  Dữ liệu: out (SimulationOutput từ Simulink)
% ============================================


%% 1. LOAD DATA FROM SIMULINK OUTPUT
logsout = out.logsout;

% Lấy từng tín hiệu
p_data = logsout.getElement('p');        % vị trí (Nt x n x 3)
v_data = logsout.getElement('v');        % vận tốc
p_ref_data = logsout.getElement('p_ref'); % vị trí tham chiếu
v_ref_data = logsout.getElement('v_ref'); % vận tốc tham chiếu
lambda_data = logsout.getElement('lambda'); % eigenvalue
Phi_data = logsout.getElement('Phi');     % obstacle potential
p_es_data = logsout.getElement('p_es');   % estimated position

% Chuyển đổi timeseries sang mảng
t = p_data.Values.Time;           % thời gian
p = p_data.Values.Data;           % vị trí [Nt x n x 3]
v = v_data.Values.Data;           % vận tốc
p_ref = p_ref_data.Values.Data;   % vị trí tham chiếu
v_ref = v_ref_data.Values.Data;   % vận tốc tham chiếu
lambda = lambda_data.Values.Data; % eigenvalue lambda2
Phi = Phi_data.Values.Data;       % obstacle potential
p_es = p_es_data.Values.Data;     % estimated position

%% Reshape
Nt = size(t,1);

p_reshaped = zeros(Nt, 10, 3);
p_es_reshaped = zeros(Nt, 10, 3);

for k = 1:Nt
    for i = 1:10
        p_reshaped(k,i,1) = p((i-1)*3+1, 1, k);
        p_reshaped(k,i,2) = p((i-1)*3+2, 1, k);
        p_reshaped(k,i,3) = p((i-1)*3+3, 1, k);

        p_es_reshaped(k,i,1) = p_es((i-1)*3+1, 1, k);
        p_es_reshaped(k,i,2) = p_es((i-1)*3+2, 1, k);
        p_es_reshaped(k,i,3) = p_es((i-1)*3+3, 1, k);
    end
end

p = p_reshaped;
p_es = p_es_reshaped;

n_drones = graph_params.n_drones;


%% Sai số khoảng cách
d_star = formation_params.d_star1;
edges = graph_params.edges;
error_history = zeros(Nt, 1);
for k = 1:Nt
    err = 0;
    for e = 1:size(edges,1)
        i = edges(e,1);
        j = edges(e,2);
        dij = norm(squeeze(p(k,i,:) - p(k,j,:)));
        err = err + (dij - d_star(e))^2;
    end
    error_history(k) = sqrt(err / size(edges,1));
end

%% ============================================
%  3. VẼ HÌNH
% ============================================

fprintf('=== PLOTTING FIGURES ===\n');

colors = lines(n_drones);

% % Figure 2: XY Projection
% %% Figure: XY Projection - Two Phases (Subplot)
figure('Name', 'XY Projection - Both Phases', 'Position', [100, 100, 1200, 600]);

% ==================== SUBPLOT 1: PHA 1 (Circular) ====================
subplot(1,2,1);
hold on; grid on;

% Vẽ quỹ đạo pha 1
for i = 1:n_drones
    plot3(p(1:501, i, 1), p(1:501, i, 2), p(1:501,i,3) , 'Color', colors(i,:), 'LineWidth', 1.2, ...
         'DisplayName', sprintf('Drone %d', i));
end

% Vị trí cuối pha 1 (t = 5s)
p_mid = squeeze(p(501, :, 1:3));

% Đường nối cuối pha 1 (không hiển thị trong legend)
for e = 1:size(edges, 1)
    i = edges(e, 1);
    j = edges(e, 2);
    plot3([p_mid(i,1), p_mid(j,1)], [p_mid(i,2), p_mid(j,2)], [p_mid(i,3), p_mid(j,3)], ...
         'k-', 'LineWidth', 1.2, 'Color', [0.4, 0.4, 0.4], 'HandleVisibility', 'off');
end

% Đánh dấu drone tại cuối pha 1
for i = 1:n_drones
    scatter3(p_mid(i,1), p_mid(i,2), p_mid(i,3), 60, colors(i,:), 'filled', ...
            'MarkerEdgeColor', 'k', 'LineWidth', 0.5, 'HandleVisibility', 'off');
    
    scatter3(p(1,i,1), p(1,i,2), p(1,i,3), 60, colors(i,:), '^', 'filled', 'HandleVisibility', 'off');
end

xlabel('X (m)'); ylabel('Y (m)');
title('Circular Formation (t = 0 to 5s)');
axis equal;

ax1 = gca;

% ==================== SUBPLOT 2: PHA 2 (Triangular) ====================
subplot(1,2,2);
hold on; grid on;

% Vẽ quỹ đạo pha 2
for i = 1:n_drones
    plot3(p(501:1001, i, 1), p(501:1001, i, 2), p(501:1001, i, 3), 'Color', colors(i,:), 'LineWidth', 1.2, ...
         'DisplayName', sprintf('Drone %d', i));
end

% Vị trí cuối pha 2 (t = 10s)
p_final = squeeze(p(end, :, 1:3));

% Đường nối cuối pha 2 (không hiển thị trong legend)
for e = 1:size(edges, 1)
    i = edges(e, 1);
    j = edges(e, 2);
    plot3([p_final(i,1), p_final(j,1)], [p_final(i,2), p_final(j,2)], [p_final(i,3), p_final(j,3)], ...
         'k-', 'LineWidth', 1.2, 'Color', [0.4, 0.4, 0.4], 'HandleVisibility', 'off');
end

% Đánh dấu drone tại cuối pha 2 (không hiển thị trong legend)
for i = 1:n_drones
    scatter3(p_final(i,1), p_final(i,2), p_final(i,3), 60, colors(i,:), 'filled', ...
            'MarkerEdgeColor', 'k', 'LineWidth', 0.5, 'HandleVisibility', 'off');
    scatter3(p(501,i,1), p(501,i,2), p(501, i, 3), 60, colors(i,:), '^', 'filled', ...
        'HandleVisibility', 'off');
end

xlabel('X (m)'); ylabel('Y (m)');
title('Triangular Formation (t = 5 to 10s)');
axis equal;
legend('Location', 'best', 'FontSize', 8);

ax2 = gca;

ax1.Position = [0.05 0.12 0.40 0.80];
ax2.Position = [0.55 0.12 0.40 0.80];

% 
% %% Figure: Altitude Error (Desired vs Actual)
% figure('Name', 'Altitude Error', 'Position', [100, 100, 1000, 600]);
% 
% % Giả sử độ cao tham chiếu là h_ref (ví dụ 5m)
% h_ref = p_ref(3,1,:);  % độ cao mong muốn (m)
% 
% % Lấy độ cao thực tế của các drone
% z_actual = p(:, :, 3);  % [Nt, n_drones]
% 
% % Tính sai số
% z_error = zeros(10, 1001);  % [Nt, n_drones]
% 
% for k = 1:Nt
%     for i = 1:10
%         z_error(i,k) = p(k,i,3) - h_ref(1,1,k);
%     end
% end
% 
% % Vẽ sai số theo thời gian
% hold on; grid on;
% for i = 1:n_drones
%     plot(t, z_error(i, :), 'Color', colors(i,:), 'LineWidth', 1.2, ...
%          'DisplayName', sprintf('Drone %d', i));
% end
% 
% % Đường zero
% yline(0, 'k--', 'LineWidth', 1.0, 'DisplayName', 'Reference');
% 
% xlabel('Time (s)'); ylabel('Altitude Error (m)');
% legend('Location', 'best', 'FontSize', 8);
% grid on;

% Figure 5
% Tính sai số ước lượng vị trí 10 agent

pes_err = zeros(Nt, 10, 3);

for k = 1:Nt
    for i = 1:10
        pes_err(k,i,1) = p_es(k,i,1) - p(k,i,1);
        pes_err(k,i,2) = p_es(k,i,2) - p(k,i,2);
        pes_err(k,i,3) = p_es(k,i,3) - p(k,i,3);
    end
end

figure('Name', 'Observator error', 'Position', [100, 100, 1200, 600]);

subplot(3,1,1);
hold on;
for i = 1:n_drones
    plot(t(1:51,1), pes_err(1:51,i,1), 'Color', colors(i,:), 'LineWidth', 1.2, ...
         'DisplayName', sprintf('Drone %d', i));
end

% Đường zero
yline(0, 'k--', 'LineWidth', 1.0, 'DisplayName', 'Reference');

xlabel('t (s)'); ylabel('$\hat{e}_x$', 'Interpreter', 'latex', 'FontSize', 16);
grid on;

subplot(3,1,2);
hold on;
for i = 1:n_drones
    plot(t(1:51,1), pes_err(1:51,i,2), 'Color', colors(i,:), 'LineWidth', 1.2, ...
         'DisplayName', sprintf('Drone %d', i));
end

% Đường zero
yline(0, 'k--', 'LineWidth', 1.0, 'DisplayName', 'Reference');

xlabel('t (s)'); ylabel('$\hat{e}_y$', 'Interpreter', 'latex', 'FontSize', 16);
grid on;

subplot(3,1,3);
hold on;
for i = 1:n_drones
    plot(t(1:51,1), pes_err(1:51,i,3), 'Color', colors(i,:), 'LineWidth', 1.2, ...
         'DisplayName', sprintf('Drone %d', i));
end

% Đường zero
yline(0, 'k--', 'LineWidth', 1.0, 'DisplayName', 'Reference');

xlabel('t (s)'); ylabel('$\hat{e}_z$', 'Interpreter', 'latex', 'FontSize', 16);
grid on;