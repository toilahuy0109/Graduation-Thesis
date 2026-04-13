%% VE TICH CHEO GIUA CAC GRADIENT (Tong cac so hang trong V_dot)
% Xet 2 drone, drone 1 tai goc toa do, drone 2 tai (x,y)

clear; clc; close all;

%% THAM SO
d_star = 3.0;       % Khoang cach mong muon
d_max = 8.0;        % Nguong mat lien ket
d_safe = 1.5;       % Nguong va cham
omega = 2.0;        % Do doc sigmoid
alpha_avoid = 1.0;
muy = (1 + d_safe^4) / d_safe^4;

%% KHAO SAT TREN MIEN 2D
x = linspace(0.5, 10, 60);
y = linspace(-6, 6, 60);
[X, Y] = meshgrid(x, y);
d = sqrt(X.^2 + Y.^2);  % Khoang cach giua 2 drone

%% ========================================================================
%  1. TINH GRADIENT CUA TUNG HAM THE (theo vi tri drone 2)
%  ========================================================================
% ∇V = (dV/dd) * (x/d, y/d)

% Formation
dV_form_dd = (d.^2 - d_star^2) .* d;
Fx_form = dV_form_dd .* (X ./ (d + 1e-6));
Fy_form = dV_form_dd .* (Y ./ (d + 1e-6));

% Connectivity
a = 1 ./ (1 + exp(-omega * (d_max - d)));
V_conn = 1 ./ (4 * a + 1e-6);
[~, dV_conn_dd] = gradient(V_conn, d(1,:), d(:,1));
Fx_conn = dV_conn_dd .* (X ./ (d + 1e-6));
Fy_conn = dV_conn_dd .* (Y ./ (d + 1e-6));

% Obstacle
beta_grid = zeros(size(d));
for i = 1:size(d,1)
    for j = 1:size(d,2)
        if d(i,j) < d_safe
            rho = 1;
        else
            rho = 0;
        end
        diff_sq = (d(i,j)^2 - d_safe^2)^2;
        beta_grid(i,j) = (1 - muy * diff_sq / (1 + diff_sq))^rho;
        beta_grid(i,j) = max(beta_grid(i,j), 0);
    end
end
V_avoid = 1 ./ (beta_grid.^alpha_avoid + 1e-6);
[~, dV_avoid_dd] = gradient(V_avoid, d(1,:), d(:,1));
Fx_avoid = dV_avoid_dd .* (X ./ (d + 1e-6));
Fy_avoid = dV_avoid_dd .* (Y ./ (d + 1e-6));

%% ========================================================================
%  2. TINH CAC TICH VO HUONG (cac so hang trong V_dot)
%  ========================================================================
% Gia su v = -∇V_form (vi v_ref = -gradV_form va he thong bam)

% Tich (∇Φ_c)^T ∇V_form
dot_conn_form = Fx_conn .* Fx_form + Fy_conn .* Fy_form;

% Tich (∇Φ_a)^T ∇V_form
dot_avoid_form = Fx_avoid .* Fx_form + Fy_avoid .* Fy_form;

% Tong (∇Φ_c + ∇Φ_a)^T ∇V_form
dot_total = dot_conn_form + dot_avoid_form;

%% ========================================================================
%  3. VE CAC TICH VO HUONG
%  ========================================================================

figure('Position', [100, 100, 1600, 1200]);

% -------------------------------------------------------------------------
%  SUBPLOT 1: (∇Φ_c)^T ∇V_form
% -------------------------------------------------------------------------
subplot(2,2,1);
contourf(X, Y, dot_conn_form, 30, 'LineStyle', 'none');
colorbar;
hold on;
% Ve duong tron d_star, d_max, d_safe
theta = linspace(0, 2*pi, 100);
plot(d_star*cos(theta), d_star*sin(theta), 'r--', 'LineWidth', 2);
plot(d_max*cos(theta), d_max*sin(theta), 'g--', 'LineWidth', 2);
plot(d_safe*cos(theta), d_safe*sin(theta), 'b--', 'LineWidth', 2);
xlabel('x (m)'); ylabel('y (m)');
title('(∇Φ_c)^T ∇V_{form}');
axis equal;
xlim([0, 10]); ylim([-6, 6]);
legend('', 'd_{star}', 'd_{max}', 'd_{safe}', 'Location', 'best');

% -------------------------------------------------------------------------
%  SUBPLOT 2: (∇Φ_a)^T ∇V_form
% -------------------------------------------------------------------------
subplot(2,2,2);
contourf(X, Y, dot_avoid_form, 30, 'LineStyle', 'none');
colorbar;
hold on;
plot(d_star*cos(theta), d_star*sin(theta), 'r--', 'LineWidth', 2);
plot(d_max*cos(theta), d_max*sin(theta), 'g--', 'LineWidth', 2);
plot(d_safe*cos(theta), d_safe*sin(theta), 'b--', 'LineWidth', 2);
xlabel('x (m)'); ylabel('y (m)');
title('(∇Φ_a)^T ∇V_{form}');
axis equal;
xlim([0, 10]); ylim([-6, 6]);

% -------------------------------------------------------------------------
%  SUBPLOT 3: (∇Φ_c + ∇Φ_a)^T ∇V_form (TONG)
% -------------------------------------------------------------------------
subplot(2,2,3);
contourf(X, Y, dot_total, 30, 'LineStyle', 'none');
colorbar;
hold on;
plot(d_star*cos(theta), d_star*sin(theta), 'r--', 'LineWidth', 2);
plot(d_max*cos(theta), d_max*sin(theta), 'g--', 'LineWidth', 2);
plot(d_safe*cos(theta), d_safe*sin(theta), 'b--', 'LineWidth', 2);
xlabel('x (m)'); ylabel('y (m)');
title('(∇Φ_c + ∇Φ_a)^T ∇V_{form} (TONG)');
axis equal;
xlim([0, 10]); ylim([-6, 6]);

% -------------------------------------------------------------------------
%  SUBPLOT 4: PHAN TICH DAU TREN CAC MIEN
% -------------------------------------------------------------------------
subplot(2,2,4);
% Tao ma tran danh dau mien
region = zeros(size(d));
region(d < d_safe) = 1;           % Vung va cham
region(d > d_max) = 2;            % Vung mat lien ket
region(d >= d_safe & d <= d_max & d < d_star) = 3;   % Vung gan (day ra)
region(d >= d_safe & d <= d_max & d >= d_star) = 4;  % Vung xa (keo vao)

% Mien dac biet: d = d_star
region(abs(d - d_star) < 0.1) = 5;

imagesc(x, y, region');
colorbar('Ticks', 1:5, 'TickLabels', {'Va cham', 'Mat lien ket', 'Gan (day)', 'Xa (keo)', 'd_{star}'});
xlabel('x (m)'); ylabel('y (m)');
title('Phan vung mien');
axis xy equal;
xlim([0, 10]); ylim([-6, 6]);

sgtitle('Phan tich cac so hang (∇Φ)^T ∇V_{form}');

%% ========================================================================
%  4. THONG KE DAU TREN CAC MIEN
%  ========================================================================

fprintf('\n========== PHAN TICH DAU CUA (∇Φ_c + ∇Φ_a)^T ∇V_{form} ==========\n');

% Vung 1: d < d_safe (va cham)
idx = d < d_safe;
fprintf('\n1. VUNG VA CHAM (d < %.2f m):\n', d_safe);
fprintf('   - (∇Φ_c)^T ∇V_form: %s\n', sign(mean(dot_conn_form(idx))));
fprintf('   - (∇Φ_a)^T ∇V_form: %s\n', sign(mean(dot_avoid_form(idx))));
fprintf('   - TONG: %s\n', sign(mean(dot_total(idx))));
if mean(dot_total(idx)) > 0
    fprintf('   => TICH DUONG → -k_d * TONG < 0 (TOT)\n');
else
    fprintf('   => TICH AM → -k_d * TONG > 0 (KHONG TOT)\n');
end

% Vung 2: d > d_max (mat lien ket)
idx = d > d_max;
fprintf('\n2. VUNG MAT LIEN KET (d > %.2f m):\n', d_max);
fprintf('   - (∇Φ_c)^T ∇V_form: %s\n', sign(mean(dot_conn_form(idx))));
fprintf('   - (∇Φ_a)^T ∇V_form: %s\n', sign(mean(dot_avoid_form(idx))));
fprintf('   - TONG: %s\n', sign(mean(dot_total(idx))));
if mean(dot_total(idx)) > 0
    fprintf('   => TICH DUONG → -k_d * TONG < 0 (TOT)\n');
else
    fprintf('   => TICH AM → -k_d * TONG > 0 (KHONG TOT)\n');
end

% Vung 3: d_safe < d < d_star
idx = (d > d_safe) & (d < d_star);
fprintf('\n3. VUNG GIUA d_safe VA d_star (%.2f < d < %.2f):\n', d_safe, d_star);
fprintf('   - (∇Φ_c)^T ∇V_form: %s\n', sign(mean(dot_conn_form(idx))));
fprintf('   - (∇Φ_a)^T ∇V_form: %s\n', sign(mean(dot_avoid_form(idx))));
fprintf('   - TONG: %s\n', sign(mean(dot_total(idx))));

% Vung 4: d_star < d < d_max
idx = (d > d_star) & (d < d_max);
fprintf('\n4. VUNG GIUA d_star VA d_max (%.2f < d < %.2f):\n', d_star, d_max);
fprintf('   - (∇Φ_c)^T ∇V_form: %s\n', sign(mean(dot_conn_form(idx))));
fprintf('   - (∇Φ_a)^T ∇V_form: %s\n', sign(mean(dot_avoid_form(idx))));
fprintf('   - TONG: %s\n', sign(mean(dot_total(idx))));

%% ========================================================================
%  5. VE VECTOR FIELD CUA TONG GRADIENT
%  ========================================================================

figure('Position', [100, 100, 800, 600]);

% Lay mau thua de vector khong bi day
step = 4;
Xq = X(1:step:end, 1:step:end);
Yq = Y(1:step:end, 1:step:end);
Fx_total_q = Fx_form(1:step:end, 1:step:end) + Fx_conn(1:step:end, 1:step:end) + Fx_avoid(1:step:end, 1:step:end);
Fy_total_q = Fy_form(1:step:end, 1:step:end) + Fy_conn(1:step:end, 1:step:end) + Fy_avoid(1:step:end, 1:step:end);

quiver(Xq, Yq, Fx_total_q, Fy_total_q, 'b', 'LineWidth', 1);
hold on;
plot(d_star*cos(theta), d_star*sin(theta), 'r-', 'LineWidth', 2);
plot(d_max*cos(theta), d_max*sin(theta), 'g-', 'LineWidth', 2);
plot(d_safe*cos(theta), d_safe*sin(theta), 'm-', 'LineWidth', 2);
xlabel('x (m)'); ylabel('y (m)');
title('Vector field: ∇V_{form} + ∇Φ_c + ∇Φ_a');
legend('Gradient', 'd_{star}', 'd_{max}', 'd_{safe}', 'Location', 'best');
axis equal;
xlim([0, 10]); ylim([-6, 6]);
grid on;