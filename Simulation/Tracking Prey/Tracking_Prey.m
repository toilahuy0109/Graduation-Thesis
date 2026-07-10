%% ============================================
%  DETERMINISTIC OPTIMAL CONTROL FOR DRONE FORMATION TRACKING
%  Cost: J = w_t*||p_c - p_t||^2 - w_c*sum(exp(-d_i^2/R_s^2)) + u'*R*u
%  Prey motion: zigzag, straight, circle, or random
%  Output: trajectories, video, and performance metrics
% ============================================

clear; clc; close all;

%% ============================================
%  1. SIMULATION PARAMETERS
% ============================================
dt = 0.05;                  % time step (s)
T_total = 40;               % total simulation time (s)
N_steps = round(T_total / dt);
time = (0:N_steps-1) * dt;

% Select prey motion type: 'zigzag', 'straight', 'circle', 'random'
prey_motion = 'random';  % Change this to test different patterns

% Create video file
video_filename = 'formation_tracking.avi';
v = VideoWriter(video_filename, 'Motion JPEG AVI');
v.FrameRate = 30;
open(v);

%% ============================================
%  2. PREY (UGV) MODEL - DETERMINISTIC TRAJECTORY
% ============================================
L_prey = 2.5;               % wheelbase (m)
v_prey = 5.0;               % constant speed (m/s)

% Initialize prey state
prey_x = zeros(N_steps, 1);
prey_y = zeros(N_steps, 1);
prey_theta = zeros(N_steps, 1);

% Set initial position
prey_x(1) = 0; prey_y(1) = 0; prey_theta(1) = 0;
if strcmp(prey_motion, 'circle')
    prey_x(1) = 10; prey_y(1) = 0; prey_theta(1) = pi/2;
end

fprintf('Generating prey trajectory (%s mode)...\n', prey_motion);

for k = 1:N_steps-1
    switch prey_motion
        case 'zigzag'
            t = k * dt;
            if mod(t, 4) < 2
                phi = deg2rad(10);
            else
                phi = deg2rad(-10);
            end
            
        case 'straight'
            phi = 0;
            
        case 'circle'
            phi = deg2rad(8);
            
        case 'random'
            sigma_phi = deg2rad(8);
            phi_min = deg2rad(-15);
            phi_max = deg2rad(15);
            a = (phi_min - 0) / sigma_phi;
            b = (phi_max - 0) / sigma_phi;
            Phi_a = normcdf(a);
            Phi_b = normcdf(b);
            u_rand = rand();
            z = norminv(Phi_a + u_rand * (Phi_b - Phi_a));
            phi = sigma_phi * z;
    end
    
    prey_theta(k+1) = prey_theta(k) + (v_prey / L_prey) * tan(phi) * dt;
    prey_x(k+1) = prey_x(k) + v_prey * cos(prey_theta(k)) * dt;
    prey_y(k+1) = prey_y(k) + v_prey * sin(prey_theta(k)) * dt;
end

%% ============================================
%  3. DRONE FORMATION PARAMETERS
% ============================================
n_drones = 10;
R_form = 10.0;
R_obs = 10.0;

% Fixed relative positions in body frame
phi_i = linspace(0, 2*pi, n_drones)';

r0 = zeros(10,2);
r0(1,:) = [0, 0];

for i = 1:n_drones-1
    r0(i+1,:) = R_form * [cos(phi_i(i)), sin(phi_i(i))];
end

%% ============================================
%  4. COST FUNCTION PARAMETERS
% ============================================
w_t = 20.0;                  % weight for center tracking
w_c = 50.0;                 % weight for Gaussian attraction
R_s = 10.0;                  % Gaussian width (m)
R_ctrl = diag([0.1, 0.1, 0.01]);  % control penalty
eta = 0.1;                  % learning rate
max_iter = 10;              % gradient descent iterations

%% ============================================
%  5. INITIAL FORMATION STATE
% ============================================
C_x = zeros(N_steps, 1);
C_y = zeros(N_steps, 1);
C_theta = zeros(N_steps, 1);

C_x(1) = -10;
C_y(1) = -5;
C_theta(1) = 0;

v_x = 0; v_y = 0; omega_c = 0;

%% ============================================
%  6. STORAGE
% ============================================
N_obs_history = zeros(N_steps, 1);
dist_history = zeros(N_steps, 1);
cost_history = zeros(N_steps, 1);
track_cost_his = zeros(N_steps,1);
u_his = zeros(N_steps,3);
gauss_his = zeros(N_steps,1);

%% ============================================
%  7. MAIN SIMULATION LOOP
% ============================================
fprintf('=== STARTING SIMULATION ===\n');

for k = 1:N_steps-1
    prey_pos = [prey_x(k); prey_y(k)];
    
    % --- GRADIENT DESCENT OPTIMIZATION ---
    for iter = 1:max_iter
        % Current drone positions
        Rmat = [cos(C_theta(k)), -sin(C_theta(k)); sin(C_theta(k)), cos(C_theta(k))];
        drones = zeros(n_drones, 2);
        for i = 1:n_drones
            drones(i, :) = [C_x(k), C_y(k)] + (Rmat * r0(i, :)')';
        end
        
        % Compute gradients
        dJ_dC = [0; 0];
        dJ_dtheta = 0;
        sum_gauss = [0; 0];
        sum_gauss_theta = 0;
        gauss_value = 0;
        
        for i = 1:n_drones
            err = drones(i, :)' - prey_pos;
            d_i = norm(err);
            gauss = exp(-d_i^2 / R_s^2);
            gauss_value = gauss_value + gauss;
            
            sum_gauss = sum_gauss + gauss * err;
            
            dR_dtheta = [-sin(C_theta(k)), -cos(C_theta(k)); cos(C_theta(k)), -sin(C_theta(k))];
            dp_dtheta = dR_dtheta * r0(i, :)';
            sum_gauss_theta = sum_gauss_theta + gauss * (err' * dp_dtheta);
        end
        
        dJ_dC = 2 * w_t * (-prey_pos + [C_x(k); C_y(k)]) + (2 * w_c / R_s^2) * sum_gauss;
        dJ_dtheta = -(2 * w_c / R_s^2) * sum_gauss_theta;
        
        % Gradients w.r.t control
        dJ_dvx = dJ_dC(1) * dt + 2 * R_ctrl(1,1) * v_x;
        dJ_dvy = dJ_dC(2) * dt + 2 * R_ctrl(2,2) * v_y;
        dJ_domega = dJ_dtheta * dt + 2 * R_ctrl(3,3) * omega_c;
        
        % Update control
        v_x = v_x - eta * dJ_dvx;
        v_y = v_y - eta * dJ_dvy;
        omega_c = omega_c - eta * dJ_domega;
        
        % Apply constraints
        % max_speed = 5.0;
        % max_omega = pi;
        % v_x = max(-max_speed, min(max_speed, v_x));
        % v_y = max(-max_speed, min(max_speed, v_y));
        % omega_c = max(-max_omega, min(max_omega, omega_c));

    end

    u_his(k+1,1) = v_x;
    u_his(k+1,2) = v_y;
    u_his(k+1,3) = omega_c;

    gauss_his(k+1) = gauss_value;
    
    % --- UPDATE FORMATION ---
    C_x(k+1) = C_x(k) + v_x * dt;
    C_y(k+1) = C_y(k) + v_y * dt;
    C_theta(k+1) = C_theta(k) + omega_c * dt;
    
    % --- METRICS ---
    Rmat = [cos(C_theta(k+1)), -sin(C_theta(k+1)); sin(C_theta(k+1)), cos(C_theta(k+1))];
    count = 0;
    cost = 0;
    for i = 1:n_drones
        drone = [C_x(k+1), C_y(k+1)] + (Rmat * r0(i, :)')';
        d = norm(drone - prey_pos');
        cost = cost - w_c*exp(-d^2/R_obs^2);
        if d <= R_obs
            count = count + 1;
        end
    end
    cost = cost + w_t*norm([C_x(k); C_y(k)] - prey_pos)^2 + [v_x, v_y, omega_c]*R_ctrl*[v_x; v_y; omega_c];
    
    N_obs_history(k+1) = count;
    dist_history(k+1) = norm([C_x(k+1); C_y(k+1)] - prey_pos);
    cost_history(k+1) = cost;
    
    if mod(k, round(N_steps/10)) == 0
        fprintf('Progress: %.0f%%, N_obs = %d\n', k/N_steps*100, count);
    end
end

fprintf('=== SIMULATION COMPLETE ===\n');

%% ============================================
%  8. PLOT RESULTS
% ============================================
% figure('Position', [50, 50, 1400, 700]);
% 
% % Trajectories
% bx1 = subplot(3,1,1);
% plot(prey_x, prey_y, 'r-', 'LineWidth', 1.5); hold on;
% plot(C_x, C_y, 'b--', 'LineWidth', 1.5);
% plot(prey_x(1), prey_y(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
% plot(C_x(1), C_y(1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
% plot(prey_x(end), prey_y(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
% plot(C_x(end), C_y(end), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
% xlabel('x (m)'); ylabel('y (m)');
% title(sprintf('Trajectories (Prey: %s)', prey_motion));
% legend('Prey', 'Formation center', 'Start', 'End', 'Location', 'best');
% grid on;
% 
% % Distance center to prey
% bx2 = subplot(3,1,2);
% plot(time, dist_history, 'b-', 'LineWidth', 1.5);
% xlabel('Time (s)'); ylabel('Distance (m)');
% title('Distance: Formation center → Prey');
% grid on;
% 
% % Formation rotation angle
% bx3 = subplot(3,1,3);
% plot(time, rad2deg(C_theta), 'c-', 'LineWidth', 1.5);
% xlabel('Time (s)'); ylabel('Angle (deg)');
% title('Formation rotation angle θ_c');
% grid on;
% 
% set([bx1 bx2 bx3], ...
%     'PositionConstraint','innerposition');

%% Monitor

% figure('Name', 'Monitor', 'Position', [50 50 1400, 900]);
% 
% % Number of observing drones
% subplot(3,1,1);
% plot(time, N_obs_history, 'g-', 'LineWidth', 1.5); hold on;
% yline(5, 'r--', 'LineWidth', 1.5);
% xlabel('Time (s)'); ylabel('Number of drone');
% title('Drones observing prey (R_{obs}=12m)');
% ylim([0, n_drones+2]); grid on;
% legend('N_{obs}', 'Threshold (5)', 'Location', 'best');
% 
% % Cost function
% subplot(3,1,2);
% plot(time, cost_history, 'm-', 'LineWidth', 1.5);
% xlabel('Time (s)'); ylabel('$L(\mathbf{p}_c,\theta,\mathbf{u},t)$', 'FontSize', 14, 'Interpreter', 'latex');
% grid on;
% 
% subplot(3,1,3);
% plot(time, gauss_his, 'Color', 'red', 'LineWidth', 1.5); hold on;
% xlabel('Time (s)'); ylabel('$\sum_{i=1}^n C_i$', 'FontSize', 14, 'Interpreter', 'latex');
% grid on;
% title('')
%% Initial State and Final State

figure('Name', 'Begin and End', 'Position', [50 50, 1400,450]);

% Initial formation snapshot
subplot(1,2,1);
hold on; theta_circ = linspace(0, 2*pi, 100);
plot(prey_x(1) + R_obs*cos(theta_circ), prey_y(1) + R_obs*sin(theta_circ), 'r--', 'LineWidth', 1);
R_init = [cos(C_theta(1)), -sin(C_theta(1)); sin(C_theta(1)), cos(C_theta(1))];
for i = 1:n_drones
    drone = [C_x(1), C_y(1)] + (R_init * r0(i,:)')';
    if i == 1
        plot(drone(1), drone(2), 'ks', 'MarkerSize', 12, 'MarkerFaceColor','k');
    else
        plot(drone(1), drone(2), 'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
    end
end
plot(prey_x(1), prey_y(1), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('x (m)'); ylabel('y (m)');
title(sprintf('Initial: N_{obs} = %d', N_obs_history(2)));
axis equal; grid on; axis square;

ax1 = gca;

% Final formation snapshot
subplot(1,2,2);
hold on;
theta_circ = linspace(0, 2*pi, 100);
obs = plot(prey_x(end) + R_obs*cos(theta_circ), prey_y(end) + R_obs*sin(theta_circ), 'r--', 'LineWidth', 1);
R_final = [cos(C_theta(end)), -sin(C_theta(end)); sin(C_theta(end)), cos(C_theta(end))];
for i = 1:n_drones
    drone = [C_x(end), C_y(end)] + (R_final * r0(i, :)')';
    if i == 1
        leader_mark = plot(drone(1), drone(2), 'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
    else
        follower_mark = plot(drone(1), drone(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    end
end
prey_mark = plot(prey_x(end), prey_y(end), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('x (m)'); ylabel('y (m)');
title(sprintf('Final: N_{obs} = %d', N_obs_history(end)));
grid on;
axis equal; axis square;

ax2 = gca;

Rview = R_obs + max(vecnorm(r0,2,2)) + 1;


xlim(ax1,[prey_x(1)-Rview, prey_x(1)+Rview]);
ylim(ax1,[prey_y(1)-Rview, prey_y(1)+Rview]);

xlim(ax2,[prey_x(end)-Rview, prey_x(end)+Rview]);
ylim(ax2,[prey_y(end)-Rview, prey_y(end)+Rview]);


legend([obs, leader_mark, follower_mark, prey_mark],{'Observation', 'Leader', 'Follower', 'Prey'}, 'Location', 'best');

%% ============================================
%  10. STATISTICS
% ============================================
fprintf('\n==================== STATISTICS ====================\n');
fprintf('Prey motion: %s\n', prey_motion);
fprintf('Total time: %.1f s\n', T_total);
fprintf('Number of drones: %d\n', n_drones);
fprintf('Observation radius: %.1f m\n', R_obs);
fprintf('\n--- Performance ---\n');
fprintf('Time with N_obs ≥ 5: %.1f%%\n', sum(N_obs_history >= 5) / N_steps * 100);
fprintf('Average N_obs: %.2f\n', mean(N_obs_history));
fprintf('Average distance (center→prey): %.2f m\n', mean(dist_history));
fprintf('Average cost: %.2f\n', mean(cost_history));
fprintf('=====================================================\n');