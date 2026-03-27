%% KIỂM TRA ẢNH HƯỞNG CỦA VIỆC THÊM TARGET LÊN TRỊ RIÊNG
clear; clc;

n_drones = 5;

% Tạo ma trận Laplace của các drone (giả sử)
L_drones = [
    3 -1 -1 -1 0;
    -1 2 -1 0 0;
    -1 -1 3 -1 0;
    -1 0 -1 2 0;
    0 0 0 0 1
];  % rank = 4

% ========================================================================
%  TRƯỜNG HỢP 1: CHƯA PHÁT HIỆN (cạnh drone-target = 0)
% ========================================================================
L_no_target = zeros(n_drones+1, n_drones+1);
L_no_target(1:n_drones, 1:n_drones) = L_drones;
% Hàng/cột của target = 0
fprintf('=== TRƯỚC KHI PHÁT HIỆN ===\n');
fprintf('Rank: %d\n', rank(L_no_target));
eigvals = eig(L_no_target);
fprintf('Các trị riêng:\n');
disp(eigvals');

% ========================================================================
%  TRƯỜNG HỢP 2: SAU KHI PHÁT HIỆN (cạnh drone-target = 1)
% ========================================================================
% Giả sử drone 3 phát hiện target
detector = 3;
L_with_target = L_no_target;
% Thêm cạnh từ detector đến target
L_with_target(detector, n_drones+1) = -1;
L_with_target(n_drones+1, detector) = -1;
% Cập nhật bậc
L_with_target(detector, detector) = L_with_target(detector, detector) + 1;
L_with_target(n_drones+1, n_drones+1) = 1;

fprintf('\n=== SAU KHI PHÁT HIỆN ===\n');
fprintf('Rank: %d\n', rank(L_with_target));
eigvals = eig(L_with_target);
fprintf('Các trị riêng:\n');
disp(eigvals');

% ========================================================================
%  SO SÁNH
% ========================================================================
fprintf('\n=== SO SÁNH ===\n');
fprintf('Số trị riêng bằng 0:\n');
fprintf('  Trước: %d\n', sum(abs(eig(L_no_target)) < 1e-6));
fprintf('  Sau: %d\n', sum(abs(eig(L_with_target)) < 1e-6));