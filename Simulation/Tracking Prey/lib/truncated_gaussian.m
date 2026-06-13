%% SO SÁNH TRUNCATED GAUSSIAN vs GAUSSIAN + CLIP
clear; clc; close all;

%% 1. THAM SỐ
mu_phi = 0.0;                       % kỳ vọng (rad)
sigma_phi_deg = 8;                  % độ lệch chuẩn (độ)
sigma_phi = deg2rad(sigma_phi_deg);
phi_min_deg = -15;                  % giới hạn dưới (độ)
phi_max_deg = 15;                   % giới hạn trên (độ)
phi_min = deg2rad(phi_min_deg);
phi_max = deg2rad(phi_max_deg);

N_samples = 10000;                  % số mẫu

%% 2. PHƯƠNG PHÁP 1: GAUSSIAN + CLIP (cách thường dùng)
rng(42);
w_clip = mu_phi + sigma_phi * randn(N_samples, 1);
w_clip = max(phi_min, min(phi_max, w_clip));

%% 3. PHƯƠNG PHÁP 2: TRUNCATED GAUSSIAN (đúng bản chất)
% Tính a, b cho truncated
a_norm = (phi_min - mu_phi) / sigma_phi;
b_norm = (phi_max - mu_phi) / sigma_phi;

Phi_a = normcdf(a_norm);
Phi_b = normcdf(b_norm);

rng(42);
u = rand(N_samples, 1);
z = norminv(Phi_a + u * (Phi_b - Phi_a));
w_trunc = mu_phi + sigma_phi * z;

%% 4. VẼ SO SÁNH HISTOGRAM
figure('Position', [100, 100, 1200, 500]);

subplot(1,2,1);
histogram(rad2deg(w_clip), 50, 'Normalization', 'pdf', 'FaceColor', [0.5, 0.5, 0.8]);
hold on;
xline(phi_min_deg, 'r--', 'LineWidth', 1.5);
xline(phi_max_deg, 'r--', 'LineWidth', 1.5);
xlabel('Steering angle \phi (deg)'); ylabel('PDF');
title('Gaussian + Clip');
ylim([0, 0.5]); grid on;

subplot(1,2,2);
histogram(rad2deg(w_trunc), 50, 'Normalization', 'pdf', 'FaceColor', [0.8, 0.5, 0.5]);
hold on;
xline(phi_min_deg, 'r--', 'LineWidth', 1.5);
xline(phi_max_deg, 'r--', 'LineWidth', 1.5);
xlabel('Steering angle \phi (deg)'); ylabel('PDF');
title('Truncated Gaussian');
ylim([0, 0.5]); grid on;

sgtitle('Comparison: Gaussian+Clip vs Truncated Gaussian');

%% 5. THỐNG KÊ
fprintf('\n=== STATISTICS ===\n');
fprintf('Gaussian + Clip:\n');
fprintf('  Mean = %.4f deg, Std = %.4f deg\n', rad2deg(mean(w_clip)), rad2deg(std(w_clip)));
fprintf('  Min = %.2f deg, Max = %.2f deg\n', rad2deg(min(w_clip)), rad2deg(max(w_clip)));
fprintf('  Samples at bounds: %.2f%%\n', sum(abs(w_clip) >= phi_max) / N_samples * 100);

fprintf('\nTruncated Gaussian:\n');
fprintf('  Mean = %.4f deg, Std = %.4f deg\n', rad2deg(mean(w_trunc)), rad2deg(std(w_trunc)));
fprintf('  Min = %.2f deg, Max = %.2f deg\n', rad2deg(min(w_trunc)), rad2deg(max(w_trunc)));
fprintf('  Samples at bounds: %.2f%%\n', sum(abs(w_trunc) >= phi_max) / N_samples * 100);