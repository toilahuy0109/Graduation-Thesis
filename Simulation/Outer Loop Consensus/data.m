clear all; clc;

%% ========================================================================
% 1. GIÁ TRỊ KHỞI TẠO
%% ========================================================================
n_drones = 10;
n_vertex = n_drones;
dim = 3;

% Vị trí ban đầu
p1_init = [0; 0; 0];
p2_init = [20; 0; 0];
p3_init = [-20; 0; 0];
p4_init = [15; 20; 0];
p5_init = [15; -20; 0];
p6_init = [30; 20; 0];
p7_init = [40; 10; 0];
p8_init = [10; 40; 0];
p9_init = [5; 50; 0];
p10_init = [45; 20; 0];

p_init = [p1_init, p2_init, p3_init, p4_init, p5_init, ...
          p6_init, p7_init, p8_init, p9_init, p10_init];

% Tham số an toàn
d_safe = 5;
d_max = 100;

%% ========================================================================
% 2. ĐỘI HÌNH MONG MUỐN - CÁC Ô VUÔNG CẠNH NHAU
%% ========================================================================
fprintf('\n=== ĐỘI HÌNH CÁC Ô VUÔNG CẠNH NHAU ===\n');

% Kích thước ô vuông (cạnh 10m)
square_size = 20;

% Cấu trúc: 2 ô vuông cạnh nhau
% Ô vuông 1: drone 1,2,3,4
% Ô vuông 2: drone 5,6,7,8
% Drone 9,10 ở vị trí khác

% Ô vuông thứ nhất (bên trái)
p_rel_star = zeros(dim, n_drones);
p_rel_star(:,1) = [0; 0; 0];           % Góc dưới trái
p_rel_star(:,2) = [square_size; 0; 0]; % Góc dưới phải
p_rel_star(:,3) = [0; square_size; 0]; % Góc trên trái
p_rel_star(:,4) = [square_size; square_size; 0]; % Góc trên phải

% Ô vuông thứ hai (bên phải, cách ô thứ nhất 10m)
offset_x = square_size + 20;  % Cách nhau 10m
p_rel_star(:,5) = [offset_x; 0; 0];                    % Góc dưới trái
p_rel_star(:,6) = [offset_x; square_size; 0];      % Góc dưới phải
p_rel_star(:,7) = [offset_x + square_size; 0; 0];          % Góc trên trái
p_rel_star(:,8) = [offset_x + square_size; square_size; 0]; % Góc trên phải

% Drone 9 và 10 tạo thành ô vuông thứ ba (phía trên, cách 10m)
offset_y = square_size + 20;
p_rel_star(:,9) = [0; offset_y; 0];                    % Góc dưới trái
p_rel_star(:,10) = [square_size; offset_y; 0];         % Góc dưới phải


% Hiển thị vị trí
fprintf('\nVị trí các drone (các ô vuông cách nhau 10m):\n');
fprintf('Drone\tX (m)\tY (m)\tZ (m)\tThuộc ô\n');
for i = 1:n_drones
    if i <= 4
        fprintf('%d\t%.2f\t%.2f\t%.2f\tÔ vuông 1\n', i, p_rel_star(1,i), p_rel_star(2,i), p_rel_star(3,i));
    elseif i <= 8
        fprintf('%d\t%.2f\t%.2f\t%.2f\tÔ vuông 2\n', i, p_rel_star(1,i), p_rel_star(2,i), p_rel_star(3,i));
    else
        fprintf('%d\t%.2f\t%.2f\t%.2f\tÔ vuông 3 (chưa đủ)\n', i, p_rel_star(1,i), p_rel_star(2,i), p_rel_star(3,i));
    end
end

%% ========================================================================
% 3. ĐỒ THỊ CỨNG VỚI CÁC CẠNH KẾT NỐI
%% ========================================================================
% Cạnh trong ô vuông 1 (4 drone tạo thành hình vuông)
edges_square1 = [
    1, 2;  % Cạnh dưới
    1, 3;  % Cạnh trái
    2, 4;  % Cạnh phải
    3, 4;  % Cạnh trên
    1, 4;  % Đường chéo chính
    2, 3   % Đường chéo phụ
];

% Cạnh trong ô vuông 2
edges_square2 = [
    5, 6;  % Cạnh dưới
    5, 7;  % Cạnh trái
    6, 8;  % Cạnh phải
    7, 8;  % Cạnh trên
    5, 8;  % Đường chéo chính
    6, 7   % Đường chéo phụ
];

% Cạnh trong ô vuông 3 (chỉ có 2 drone nên chỉ 1 cạnh)
edges_square3 = [
    9, 10   % Cạnh nối 2 drone
];

% Cạnh nối giữa các ô vuông (để đảm bảo liên thông)
edges_between = [
    1, 5;   % Nối ô 1 và ô 2
    4, 8;   % Nối ô 1 và ô 2
    3, 7;   % Nối ô 1 và ô 2
    2, 6;   % Nối ô 1 và ô 2
    1, 9;   % Nối ô 1 và ô 3
    2, 10;  % Nối ô 1 và ô 3
    5, 9;   % Nối ô 2 và ô 3
    6, 10   % Nối ô 2 và ô 3
];

% Ghép tất cả edges
edges = [edges_square1; edges_square2; edges_square3; edges_between];
n_edges = size(edges, 1);

fprintf('\n=== ĐỒ THỊ ===\n');
fprintf('Số cạnh: %d (yêu cầu tối thiểu: %d)\n', n_edges, 3*n_drones-6);

if n_edges < 3*n_drones-6
    fprintf('⚠️  Đồ thị có %d cạnh, cần thêm %d cạnh để đủ cứng\n', n_edges, 3*n_drones-6 - n_edges);
else
    fprintf('✓ Đồ thị đủ cứng trong 3D\n');
end

% Hiển thị edges
fprintf('\n=== DANH SÁCH EDGES ===\n');
fprintf('Cạnh trong ô vuông 1: %d cạnh\n', size(edges_square1,1));
fprintf('Cạnh trong ô vuông 2: %d cạnh\n', size(edges_square2,1));
fprintf('Cạnh trong ô vuông 3: %d cạnh\n', size(edges_square3,1));
fprintf('Cạnh nối giữa các ô: %d cạnh\n', size(edges_between,1));
fprintf('Tổng: %d cạnh\n', n_edges);

% Danh sách neighbors
neighbors = cell(n_drones, 1);
for i = 1:n_drones
    neighbors{i} = [];
    for e = 1:n_edges
        if edges(e,1) == i
            neighbors{i} = [neighbors{i}, edges(e,2)];
        elseif edges(e,2) == i
            neighbors{i} = [neighbors{i}, edges(e,1)];
        end
    end
    neighbors{i} = sort(unique(neighbors{i}));
end

fprintf('\n=== DANH SÁCH NEIGHBORS ===\n');
for i = 1:n_drones
    if ~isempty(neighbors{i})
        fprintf('Drone %d neighbors (%d): ', i, length(neighbors{i}));
        fprintf('%d ', neighbors{i});
        fprintf('\n');
    end
end

% Tính khoảng cách mong muốn
d_star = zeros(n_edges, 1);
fprintf('\n=== KHOẢNG CÁCH MONG MUỐN ===\n');
for e = 1:n_edges
    i = edges(e,1);
    j = edges(e,2);
    d_star(e) = norm(p_rel_star(:,i) - p_rel_star(:,j));
    if e <= 20
        fprintf('  d%d%d* = %.2f m\n', i, j, d_star(e));
    end
end
if n_edges > 20
    fprintf('  ... và %d cạnh khác\n', n_edges-20);
end

%% ========================================================================
% 4. MA TRẬN Q (CHO HOLD CONNECTION)
%% ========================================================================
Q = randn(n_drones, n_drones-1);
for i = 1:n_drones-1
    % Trừ đi thành phần theo vector đơn vị
    Q(:,i) = Q(:,i) - (1/n_drones)*(sum(Q(:,i))) * ones(n_drones,1);
    % Gram-Schmidt orthogonalization
    for j = 1:i-1
        Q(:,i) = Q(:,i) - (Q(:,j)' * Q(:,i)) * Q(:,j);
    end
    % Chuẩn hóa
    if norm(Q(:,i)) > 1e-10
        Q(:,i) = Q(:,i) / norm(Q(:,i));
    end
end

fprintf('\n=== MA TRẬN Q ===\n');
fprintf('Kích thước Q: %d x %d\n', size(Q,1), size(Q,2));
fprintf('Kiểm tra Q''*Q (phải là ma trận đơn vị):\n');
disp(Q' * Q);
fprintf('Kiểm tra sum(Q) (mỗi cột phải gần bằng 0):\n');
disp(sum(Q));

%% ========================================================================
% 5. THỐNG KÊ ĐỘI HÌNH
%% ========================================================================
fprintf('\n=== THỐNG KÊ ĐỘI HÌNH Ô VUÔNG ===\n');

% Khoảng cách đến leader (drone 1)
fprintf('\nKhoảng cách đến leader (drone 1):\n');
for i = 2:n_drones
    dist = norm(p_rel_star(:,i) - p_rel_star(:,1));
    fprintf('Drone %d: %.2f m\n', i, dist);
end

% Thống kê khoảng cách
all_dist = [];
for i = 1:n_drones
    for j = i+1:n_drones
        all_dist = [all_dist; norm(p_rel_star(:,i) - p_rel_star(:,j))];
    end
end

fprintf('\nThống kê khoảng cách giữa các drone:\n');
fprintf('  Trung bình: %.2f m\n', mean(all_dist));
fprintf('  Độ lệch chuẩn: %.2f m\n', std(all_dist));
fprintf('  Nhỏ nhất: %.2f m\n', min(all_dist));
fprintf('  Lớn nhất: %.2f m\n', max(all_dist));

% Kiểm tra với d_safe và d_max
fprintf('\n=== KIỂM TRA AN TOÀN ===\n');
fprintf('d_safe = %.2f m\n', d_safe);
fprintf('d_max = %.2f m\n', d_max);

if min(all_dist) < d_safe
    fprintf('⚠️  CẢNH BÁO: Khoảng cách nhỏ nhất (%.2f m) < d_safe (%.2f m)\n', min(all_dist), d_safe);
else
    fprintf('✓ Khoảng cách nhỏ nhất (%.2f m) > d_safe (%.2f m)\n', min(all_dist), d_safe);
end

if max(all_dist) > d_max
    fprintf('⚠️  CẢNH BÁO: Khoảng cách lớn nhất (%.2f m) > d_max (%.2f m)\n', max(all_dist), d_max);
else
    fprintf('✓ Khoảng cách lớn nhất (%.2f m) < d_max (%.2f m)\n', max(all_dist), d_max);
end

% Khoảng cách trong từng ô vuông
fprintf('\nKhoảng cách trong ô vuông 1 (cạnh = %.2f m):\n', square_size);
fprintf('  Cạnh: %.2f m\n', square_size);
fprintf('  Đường chéo: %.2f m\n', square_size*sqrt(2));

fprintf('\nKhoảng cách giữa các ô vuông:\n');
fprintf('  Ô 1 và ô 2: cách nhau %d m\n', 10);
fprintf('  Ô 1 và ô 3: cách nhau %d m\n', 10);