clear all, clc;
% 1. GIÁ TRỊ KHỞI TẠO
%% ========================================================================
n_drones = 5;
n_vertex = 5;
dim = 3;

p1_init = [0; 0; 0];
p2_init = [20; 0; 0];
p3_init = [-20; 0; 0];
p4_init = [15; 20; 0];
p5_init = [15; -20; 0];

%% ========================================================================
%  2. ĐỘI HÌNH MONG MUỐN
%% ========================================================================
% Drone 1 (leader) ở đỉnh, 4 followers ở đáy hình vuông
L = 6;       % khoảng cách từ tâm đáy đến các góc
H = 8;       % chiều cao của hình chóp

% Vị trí mong muốn so với leader
p_rel_star = zeros(dim, n_drones);
p_rel_star(:,1) = [0; 0; 0];           % leader ở đỉnh
p_rel_star(:,2) = [L; L; -H];          % follower 2
p_rel_star(:,3) = [L; -L; -H];         % follower 3
p_rel_star(:,4) = [-L; L; -H];         % follower 4
p_rel_star(:,5) = [-L; -L; -H];        % follower 5

%% ========================================================================
%  3. ĐỒ THỊ CỨNG 9 CẠNH (3n-6 = 9)
%% ========================================================================
edges = [1 2; 1 3; 1 4; 1 5;   % leader với all followers (4 cạnh)
         2 3; 2 4; 2 5; 3 4;         % tam giác giữa 2,3,4 (3 cạnh)
         3 5; 4 5];             % nối 5 với 3 và 4 (2 cạnh)

m = size(edges, 1);
fprintf('Số cạnh: %d (yêu cầu tối thiểu: %d)\n', m, 3*n_drones-6);

% Kiểm tra
if m < 3*n_drones-6
    warning('Đồ thị không đủ cứng! Cần %d cạnh, chỉ có %d', 3*n_drones-6, m);
else
    fprintf('Đồ thị đủ cứng trong 3D\n');
end

% Danh sách neighbors
neighbors = cell(n_drones, 1);
for i = 1:n_drones
    neighbors{i} = [];
    for e = 1:m
        if edges(e,1) == i
            neighbors{i} = [neighbors{i}, edges(e,2)];
        elseif edges(e,2) == i
            neighbors{i} = [neighbors{i}, edges(e,1)];
        end
    end
    neighbors{i} = sort(unique(neighbors{i}));
    fprintf('Drone %d neighbors: ', i);
    fprintf('%d ', neighbors{i});
    fprintf('\n');
end

% Tính khoảng cách mong muốn
d_star = zeros(m, 1);
fprintf('\n=== KHOẢNG CÁCH MONG MUỐN ===\n');
for e = 1:m
    i = edges(e,1); j = edges(e,2);
    d_star(e) = norm(p_rel_star(:,i) - p_rel_star(:,j));
    fprintf('  d%d%d* = %.2f m\n', i, j, d_star(e));
end

%% ========================================================================
% 4. TẠO MA TRẬN Q (HOLD CONNECTION)
%% ========================================================================
Q = randn(n_drones, n_drones-1);
for i = 1:n_drones-1
    Q(:,i) = Q(:,i) - (1/n_drones)*(ones(1,n_drones) * Q(:,i)) * ones(n_drones,1);
    for j = 1:i-1
        Q(:,i) = Q(:,i) - (Q(:,j)' * Q(:,i)) * Q(:,j);
    end
    Q(:,i) = Q(:,i) / norm(Q(:,i));
end


%% ========================================================================
% 5. CÁC HÀM PHỤ
%% ========================================================================
function A = adjacency_sigmoid(P, delta, omega)
    pos = cell(n_drones,1);
    for i = 1:n_drones
        pos{i} = P((i-1)*3+1:i*3);
    end
    A = zeros(n_drones);
    for i = 1:n_drones
        for j = i+1:n_drones
            d_ij = norm(pos{i} - pos{j});
            a_ij = 1 / (1 + exp(-omega * (delta - d_ij)));
            A(i,j) = a_ij;
            A(j,i) = a_ij;
        end
    end
end

function L = laplacian_from_adjacency(A)
    D = diag(sum(A,2));
    L = D - A;
end

function M = compute_M(P, delta, omega, Q)
    A = adjacency_sigmoid(P, delta, omega);
    L = laplacian_from_adjacency(A);
    M = Q'*L*Q;
end
%% ========================================================================