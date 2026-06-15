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
u_data = logsout.getElement('u');
grad_data = logsout.getElement('grad');

% Chuyển đổi timeseries sang mảng
t = p_data.Values.Time;           % thời gian
p = p_data.Values.Data;           % vị trí [Nt x n x 3]
v = v_data.Values.Data;           % vận tốc
p_ref = p_ref_data.Values.Data;   % vị trí tham chiếu
v_ref = v_ref_data.Values.Data;   % vận tốc tham chiếu
lambda = lambda_data.Values.Data; % eigenvalue lambda2
Phi = Phi_data.Values.Data;       % obstacle potential
p_es = p_es_data.Values.Data;     % estimated position
u = u_data.Values.Data;
grad = grad_data.Values.Data;

%% Reshape
Nt = size(t,1);

p_reshaped = zeros(Nt, 10, 3);
p_es_reshaped = zeros(Nt, 10, 3);
u_reshaped = zeros(Nt, 10, 3);
grad_reshaped = zeros(Nt, 10, 3);

for k = 1:Nt
    for i = 1:10
        p_reshaped(k,i,1) = p((i-1)*3+1, 1, k);
        p_reshaped(k,i,2) = p((i-1)*3+2, 1, k);
        p_reshaped(k,i,3) = p((i-1)*3+3, 1, k);

        p_es_reshaped(k,i,1) = p_es((i-1)*3+1, 1, k);
        p_es_reshaped(k,i,2) = p_es((i-1)*3+2, 1, k);
        p_es_reshaped(k,i,3) = p_es((i-1)*3+3, 1, k);

        if i == 1
            u_reshaped(Nt, i, 1) = 0;
            u_reshaped(Nt, i, 2) = 0;
            u_reshaped(Nt, i, 3) = 0;

            grad_reshaped(Nt, i, 1) = 0;
            grad_reshaped(Nt, i, 2) = 0;
            grad_reshaped(Nt, i, 3) = 0;
        else
            u_reshaped(k,i,1) = u((i-1)*3+1, 1, k);
            u_reshaped(k,i,2) = u((i-1)*3+2, 1, k);
            u_reshaped(k,i,3) = u((i-1)*3+3, 1, k);

            grad_reshaped(k,i,1) = grad((i-1)*3+1, 1, k);
            grad_reshaped(k,i,2) = grad((i-1)*3+2, 1, k);
            grad_reshaped(k,i,3) = grad((i-1)*3+3, 1, k);
        end
    end
end

p = p_reshaped;
p_es = p_es_reshaped;
u = u_reshaped;
grad = grad_reshaped;

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

% % % Figure 2: XY Projection
%% Figure: XY Projection - Two Phases (Subplot)
% figure('Name', 'XY Projection - Both Phases', 'Position', [100, 100, 1200, 600]);
% 
% text(0.5, -0.15, 'Number of links = 38', ...
%     'Units','normalized', ...
%     'HorizontalAlignment', 'center');
% % ==================== SUBPLOT 1: PHA 1 (Circular) ====================
% subplot(1,2,1);
% hold on; grid on;
% 
% % Vẽ quỹ đạo pha 1
% for i = 1:n_drones
%     plot3(p(1:501, i, 1), p(1:501, i, 2), p(1:501,i,3) , 'Color', colors(i,:), 'LineWidth', 1.2, ...
%          'DisplayName', sprintf('Drone %d', i));
% end
% 
% % Vị trí cuối pha 1 (t = 5s)
% p_mid = squeeze(p(501, :, 1:3));
% 
% % Đường nối cuối pha 1 (không hiển thị trong legend)
% for e = 1:size(edges, 1)
%     i = edges(e, 1);
%     j = edges(e, 2);
%     plot3([p_mid(i,1), p_mid(j,1)], [p_mid(i,2), p_mid(j,2)], [p_mid(i,3), p_mid(j,3)], ...
%          'k-', 'LineWidth', 1.2, 'Color', [0.4, 0.4, 0.4], 'HandleVisibility', 'off');
% end
% 
% % Đánh dấu drone tại cuối pha 1
% for i = 1:n_drones
%     scatter3(p_mid(i,1), p_mid(i,2), p_mid(i,3), 60, colors(i,:), 'filled', ...
%             'MarkerEdgeColor', 'k', 'LineWidth', 0.5, 'HandleVisibility', 'off');
% 
%     scatter3(p(1,i,1), p(1,i,2), p(1,i,3), 60, colors(i,:), '^', 'filled', 'HandleVisibility', 'off');
% end
% 
% xlabel('X (m)'); ylabel('Y (m)');
% title('Circular Formation (t = 0 to 5s)');
% axis equal;
% 
% ax1 = gca;
% 
% % ==================== SUBPLOT 2: PHA 2 (Triangular) ====================
% subplot(1,2,2);
% hold on; grid on;
% 
% % Vẽ quỹ đạo pha 2
% for i = 1:n_drones
%     plot3(p(501:1001, i, 1), p(501:1001, i, 2), p(501:1001, i, 3), 'Color', colors(i,:), 'LineWidth', 1.2, ...
%          'DisplayName', sprintf('Drone %d', i));
% end
% 
% % Vị trí cuối pha 2 (t = 10s)
% p_final = squeeze(p(end, :, 1:3));
% 
% % Đường nối cuối pha 2 (không hiển thị trong legend)
% for e = 1:size(edges, 1)
%     i = edges(e, 1);
%     j = edges(e, 2);
%     plot3([p_final(i,1), p_final(j,1)], [p_final(i,2), p_final(j,2)], [p_final(i,3), p_final(j,3)], ...
%          'k-', 'LineWidth', 1.2, 'Color', [0.4, 0.4, 0.4], 'HandleVisibility', 'off');
% end
% 
% % Đánh dấu drone tại cuối pha 2 (không hiển thị trong legend)
% for i = 1:n_drones
%     scatter3(p_final(i,1), p_final(i,2), p_final(i,3), 60, colors(i,:), 'filled', ...
%             'MarkerEdgeColor', 'k', 'LineWidth', 0.5, 'HandleVisibility', 'off');
%     scatter3(p(501,i,1), p(501,i,2), p(501, i, 3), 60, colors(i,:), '^', 'filled', ...
%         'HandleVisibility', 'off');
% end
% 
% xlabel('X (m)'); ylabel('Y (m)');
% title('Triangular Formation (t = 5 to 10s)');
% axis equal;
% legend('Location', 'best', 'FontSize', 8);
% 
% ax2 = gca;
% 
% view(ax1,[45 25]);
% view(ax2,[45 25]);
% 
% ax1.Position = [0.05 0.12 0.40 0.80];
% ax2.Position = [0.55 0.12 0.40 0.80];


%% Figure: Distance Error (Desired vs Actual)
% figure('Name', 'Altitude Error', 'Position', [100, 100, 1500, 400]);
% 
% % Tinh toan sai so khoang cach
% dist_err = zeros(Nt, n_drones);
% for time = 1:Nt
%     for k = 1:n_drones
%         error = 0;
%         for e = 1:n_edges
%             i = edges(e,1);
%             j = edges(e,2);
%             if k==i || k==j
%                 dist = norm(squeeze(p(time, i, :) - p(time, j, :)));
%                 if time <= 501
%                     error = error + (dist - formation_params.d_star1(e))^2;
%                 elseif time > 501
%                     error = error + (dist - formation_params.d_star2(e))^2;
%                 end
%             end
%         end
%         dist_err(time, k) = sqrt(error);
%     end
% end
% 
% subplot(2,1,1);
% hold on; grid on;
% 
% for i = 1:n_drones
%     plot(t(:,1), dist_err(:,i), 'Color', colors(i,:), 'LineWidth',1.2, 'DisplayName', sprintf('Drone %d', i));
% end
% 
% 
% % Đường zero
% yline(0, 'k--', 'LineWidth', 1.0, 'DisplayName', 'Reference');
% 
% xlabel('t (s)'); ylabel('$\|d_{ij} - d^\star\|$', 'Interpreter', 'latex', 'FontSize', 16);
% legend('Location', 'best', 'FontSize', 8, 'NumColumns', 2);
% 
% subplot(2,1,2);
% hold on; grid on;
% 
% height_err = zeros(Nt, n_drones);
% 
% for k = 1:Nt
%     for i = 1:n_drones
%         height_err(k,i) = norm(squeeze(p(k,i,3) - p_ref(3,1,k)));
%     end
% end
% 
% for i = 1:n_drones
%     plot(t(:,1), height_err(:,i), 'Color', colors(i,:), 'LineWidth', 1.2, 'DisplayName', sprintf('Drone %d', i));
% end
% 
% 
% % Đường zero
% yline(0, 'k--', 'LineWidth', 1.0, 'DisplayName', 'Reference');
% 
% xlabel('t (s)'); ylabel('$\|e_h\|$', 'Interpreter', 'latex', 'FontSize', 16);
% legend('Location', 'best', 'FontSize', 8, 'NumColumns', 2);

%% Figure 5
% % Tính sai số ước lượng vị trí 10 agent
% 
% pes_err = zeros(Nt, 10);
% 
% for k = 1:Nt
%     for i = 1:10
%         pes_err(k,i) = sqrt((p_es(k,i,1)-p(k,i,1))^2 + (p_es(k,i,2)-p(k,i,2))^2 + (p_es(k,i,3)-p(k,i,3))^2);
%     end
% end
% 
% figure('Name', 'Observator error', 'Position', [100, 100, 1200, 250]);
% hold on;
% for i = 1:n_drones
%     plot(t(1:151,1), pes_err(1:151,i), 'Color', colors(i,:), 'LineWidth', 1.2, ...
%          'DisplayName', sprintf('Drone %d', i));
% end
% 
% % Đường zero
% yline(0, 'k--', 'LineWidth', 1.0, 'DisplayName', 'Reference');
% 
% xlabel('t (s)'); ylabel('$\|\tilde{\mathbf{z}}_1\|$', 'Interpreter', 'latex', 'FontSize', 16);
% grid on;
% legend('Location', 'best', 'FontSize', 8, 'NumColumns', 2);

%% Figure: Min and Max distance with safe and unsafe distance
% figure('Name', 'Safe Distance and Potential Function for Connection', 'Position', [100, 100, 1500, 500]);
% 
% min_dist = zeros(Nt,1);
% max_dist = zeros(Nt,1);
% u_min = zeros(Nt, 1);
% 
% for k = 1:Nt
%     min_dist(k,1) = norm(squeeze(p(k,1,:) - p(k,2,:)));
%     max_dist(k,1) = min_dist(k,1);
%     for d = 1:n_drones
%         for e = 1:n_edges
%             i = graph_params.edges(e,1);
%             j = graph_params.edges(e,2);
% 
%             dist = norm(squeeze(p(k,i,:) - p(k,j,:)));
% 
%             if dist < min_dist(k,1)
%                 min_dist(k,1) = dist;
%             end
%             if dist > max_dist(k,1)
%                 max_dist(k,1) = dist;
%             end
%         end
%     end
% end
% 
% subplot(2,1,1);
% hold on; grid on;
% 
% plot(t(:,1), min_dist(:,1), 'Color', colors(i,:), 'LineWidth', 1.2);
% 
% % Safe distance
% yline(3, 'k--', 'LineWidth', 1.0, 'DisplayName', 'Reference');
% 
% xlabel('t (s)'); ylabel('$min \| d_i - d_j \|$', 'Interpreter', 'latex', 'FontSize', 16);
% legend('Location', 'best', 'FontSize', 8);
% 
% title('Min Distance between 2 of 10 drones', 'FontSize', 12);
% 
% subplot(2,1,2);
% hold on, grid on;
% 
% plot(t(:,1), max_dist(:,1), 'Color', colors(1,:), 'LineWidth', 1.2);
% 
% % Max distance
% yline(100, 'k--', 'LineWidth', 1.0, 'DisplayName', 'Reference');
% 
% xlabel('t (s)'); ylabel('$max \| d_i - d_j \|$', 'Interpreter', 'latex', 'FontSize', 16);
% 
% title('Max Distance between 2 of 10 drones', 'FontSize', 12);

%% Figure : Grad

% figure('Name', 'Sum of Grad', 'Position', [100 100 1500 250]);
% hold on; grid on;
% 
% sum_grad = zeros(Nt, 3);
% 
% for k = 1:Nt
%     for i = 1:n_drones
%         sum_grad(Nt,1) = sum_grad(Nt,1) + grad(k,i,1);
%         sum_grad(Nt,2) = sum_grad(Nt,2) + grad(k,i,2);
%         sum_grad(Nt,3) = sum_grad(Nt,3) + grad(k,i,3);
%     end
% end
% 
% sum_grad(Nt,:) = [0, 0, 0];
% 
% hold on; grid on;
% 
% for i = 1:3
%     plot(t(:,1), sum_grad(:,i), 'Color', colors(i,:), 'LineWidth', 1.2);
% end
% 
% xlabel('t (s)'); ylabel('$ \sum \nabla_{\mathbf{p}_i} V_2 $', 'Interpreter', 'latex', 'FontSize', 16);
% legend('$\sum \nabla_{\mathbf{p}_i} V_2 (1)$', '$\sum \nabla_{\mathbf{p}_i} V_2 (2)$', '$\sum \nabla_{\mathbf{p}_i} V_2 (1)$', 'Interpreter', 'latex', 'FontSize', 12);






