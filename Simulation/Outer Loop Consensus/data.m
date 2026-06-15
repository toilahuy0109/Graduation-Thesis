clear all; clc;

%% ========================================================================
% 1. GIÁ TRỊ KHỞI TẠO
%% ========================================================================

params = struct();

%% Parameters

graph_params = struct();
graph_params.n_drones = 10;
graph_params.n_vertex = graph_params.n_drones;
graph_params.dim = 3;


leader_params = struct();
leader_params.gamma_leader = 5;
leader_params.beta_leader = 20;


formation_params = struct();
formation_params.alpha = 0.1;
formation_params.K2 = 25;
formation_params.beta = 5;
formation_params.kp = 10;
formation_params.kd = 2;


avoid_params = struct();
avoid_params.k = 0.3;
avoid_params.alpha = 2;
avoid_params.d_safe = 3;


conn_params = struct();
conn_params.omega = 20;
conn_params.epsilon = 0.4547;
conn_params.alpha = 4;
conn_params.d_max = 100;

estimation_params = struct();
estimation_params.mu = 0.2;

prior_params.K1 = 5;
prior_params.k_form = 2;
prior_params.k_conn = 1;
prior_params.k_avoid = 5;
prior_params.k_height = 50;



% Vị trí ban đầu (random hoặc cố định)
% Có thể dùng random hoặc set cụ thể
use_random_initial = false;  % false để dùng vị trí cố định

if use_random_initial
    rng(42);
    p1_init = [0; 0; 0] + 2*randn(3,1);
    p2_init = [15; 10; 0] + 2*randn(3,1);
    p3_init = [-10; 15; 0] + 2*randn(3,1);
    p4_init = [5; -12; 0] + 2*randn(3,1);
    p5_init = [-5; -8; 0] + 2*randn(3,1);
    p6_init = [12; -5; 0] + 2*randn(3,1);
    p7_init = [-12; 5; 0] + 2*randn(3,1);
    p8_init = [8; 12; 0] + 2*randn(3,1);
    p9_init = [-8; -10; 0] + 2*randn(3,1);
    p10_init = [3; 15; 0] + 2*randn(3,1);
else
    % Vị trí ban đầu cố định
    p1_init = [0; 0; 0];
    p2_init = [10; 0; 0];
    p3_init = [0; 10; 0];
    p4_init = [-10; 0; 0];
    p5_init = [0; -10; 0];
    p6_init = [7; 7; 0];
    p7_init = [-7; 7; 0];
    p8_init = [-7; -7; 0];
    p9_init = [7; -7; 0];
    p10_init = [9; 5; 0];
end

p_init = [p1_init, p2_init, p3_init, p4_init, p5_init, ...
          p6_init, p7_init, p8_init, p9_init, p10_init];

%% ========================================================================
% 2. ĐỘI HÌNH MONG MUỐN - VÒNG TRÒN VỚI DRONE 1 Ở TÂM
%% ========================================================================
fprintf('\n=== ĐỘI HÌNH VÒNG TRÒN ===\n');

% Bán kính vòng tròn
R = 12;              % Bán kính vòng tròn (m)
H = 3;               % Độ cao của các drone trên vòng tròn

% Drone 1 ở tâm
p_rel_star = zeros(graph_params.dim, graph_params.n_drones);
p_rel_star(:,1) = [0; 0; 0];  % Drone 1 (leader) ở tâm

% Tạo 9 góc đều nhau cho 9 drone còn lại
angles = linspace(0, 2*pi, graph_params.n_drones)';  % 10 góc
angles = angles(1:end-1);                % Lấy 9 góc đầu (bỏ góc cuối trùng góc đầu)

% Phân bố đều trên vòng tròn với độ cao thay đổi
for i = 2:graph_params.n_drones
    theta = angles(i-1);
    % Vị trí trên vòng tròn với độ cao thay đổi theo sin để tạo hiệu ứng 3D
    p_rel_star(:,i) = [R * cos(theta); R * sin(theta); H * sin(2*theta)];
end


%% ========================================================================
% 3. ĐỒ THỊ CỨNG VỚI CÁC CẠNH KẾT NỐI (ĐỦ 24 CẠNH)
%% ========================================================================
% Cạnh từ drone trung tâm (drone 1) đến tất cả các drone khác
edges_center = [];
for i = 2:graph_params.n_drones
    edges_center = [edges_center; 1, i];
end

% Cạnh vòng tròn giữa các drone trên vòng tròn (kết nối các drone lân cận)
edges_circle = [];
for i = 2:graph_params.n_drones-1
    edges_circle = [edges_circle; i, i+1];
end
edges_circle = [edges_circle; graph_params.n_drones, 2];  % Kết nối drone cuối với drone đầu

% Cạnh chéo để tăng độ cứng (đảm bảo đủ 24 cạnh)
edges_diagonal = [
    2, 4;  3, 5;  4, 6;  5, 7;  6, 8;  7, 9;  8, 10;  9, 2;
    2, 5;  3, 6;  4, 7;  5, 8;  6, 9;  7, 10;  8, 2;  9, 3; 
    2, 7;  3, 8;  4, 9;  5, 10
];

% Ghép tất cả edges


graph_params.edges = [edges_center; edges_circle; edges_diagonal];
n_edges = size(graph_params.edges, 1);

fprintf('\n=== ĐỒ THỊ ===\n');
fprintf('Số cạnh: %d (yêu cầu tối thiểu: %d)\n', n_edges, 3*graph_params.n_drones-6);

if n_edges < 3*graph_params.n_drones-6
    fprintf('Đồ thị có %d cạnh, cần thêm %d cạnh để đủ cứng\n', n_edges, 3*graph_params.n_drones-6 - n_edges);
else
    fprintf('Đồ thị đủ cứng trong 3D\n');
end

% Hiển thị edges
fprintf('\n=== DANH SÁCH EDGES ===\n');
fprintf('Tổng: %d cạnh\n', n_edges);


% Tính khoảng cách mong muốn

formation_params.d_star1 = zeros(n_edges, 1);

for e = 1:n_edges
    i = graph_params.edges(e,1);
    j = graph_params.edges(e,2);
    formation_params.d_star1(e) = norm(p_rel_star(:,i) - p_rel_star(:,j));
end

p_triangle = zeros(10, 3);

formation_params.d_star2 = zeros(n_edges, 1);

% Khoang cach giua cac drone (canh tam giac deu)
a = 10;  % met

% Tao luoi tam giac
idx = 1;
for row = 1:4  % 4 hang
    y = -(row-1) * a * sqrt(3)/2;  % toa do y (truc dung)
    for col = 1:row
        x = (col - (row+1)/2) * a;
        p_triangle(idx,:) = [x, y, 0];
        idx = idx + 1;
    end
end

for e = 1:n_edges
    i = graph_params.edges(e, 1);
    j = graph_params.edges(e, 2);
    formation_params.d_star2(e) = norm(p_triangle(i, :) - p_triangle(j, :));
end

%% ========================================================================
% 4. MA TRẬN Q (CHO HOLD CONNECTION)
%% ========================================================================
conn_params.Q = randn(graph_params.n_drones, graph_params.n_drones-1);
for i = 1:graph_params.n_drones-1
    % Trừ đi thành phần theo vector đơn vị
    conn_params.Q(:,i) = conn_params.Q(:,i) - (1/graph_params.n_drones)*(sum(conn_params.Q(:,i))) * ones(graph_params.n_drones,1);
    % Gram-Schmidt orthogonalization
    for j = 1:i-1
        conn_params.Q(:,i) = conn_params.Q(:,i) - (conn_params.Q(:,j)' * conn_params.Q(:,i)) * conn_params.Q(:,j);
    end
    % Chuẩn hóa
    if norm(conn_params.Q(:,i)) > 1e-10
        conn_params.Q(:,i) = conn_params.Q(:,i) / norm(conn_params.Q(:,i));
    end
end

fprintf('\n=== MA TRẬN Q ===\n');
fprintf('Kích thước Q: %d x %d\n', size(conn_params.Q,1), size(conn_params.Q,2));


global edges
edges = graph_params.edges;

%% ========================================================================
% 7. CẤU HÌNH TẤN CÔNG
%% ========================================================================
attack_scenario = 1;

attack_config = struct();
attack_config.start_time = 10;
attack_config.end_time = 25;
attack_config.target_drones = [2, 5, 8];

% Parameters
attack_config.fdi_bias = [8; 6; 3];
attack_config.fdi_scale = 2.0;
attack_config.dos_probability = 0.4;
attack_config.gps_bias = [10; 8; 5];
attack_config.gps_drift = [0.5; 0.5; 0.2];
attack_config.sensor_bias = 5.0;
attack_config.sensor_noise = 2.0;

%% ========================================================================
% 5. TẠO BUS VÀ PARAMETER VỚI KÍCH THƯỚC CỐ ĐỊNH (QUAN TRỌNG)
%% ========================================================================

% Gom tất cả vào một struct chính
params.graph_params = graph_params;
params.leader_params = leader_params;
params.formation_params = formation_params;
params.avoid_params = avoid_params;
params.conn_params = conn_params;
params.estimation_params = estimation_params;
params.prior_params = prior_params;

% --- ÉP BUỘC KÍCH THƯỚC CỦA CÁC MẢNG TRONG STRUCT ---
% Đây là chìa khóa để trị lỗi "variable-size"
params.graph_params.edges = double(params.graph_params.edges);
params.graph_params.n_drones = double(params.graph_params.n_drones);
params.graph_params.dim = double(params.graph_params.dim);
params.formation_params.d_star1 = double(params.formation_params.d_star1);
params.formation_params.d_star2 = double(params.formation_params.d_star2);
params.conn_params.Q = double(params.conn_params.Q);

% --- Tạo Bus từ struct đã được "cố định hóa" ---
busInfo = Simulink.Bus.createObject(params);
% MATLAB thường đặt tên Bus là 'slBus1', 'slBus2',... Hãy lấy đúng tên
bus_name = busInfo.busName; 
assignin('base', 'params_bus', eval(bus_name));
fprintf('Bus name: %s\n', bus_name);

% --- Tạo Simulink.Parameter trong Model Workspace ---
model_name = 'Formation_Based_Distance';
hws = get_param(model_name, 'ModelWorkspace');
hws.clear();

% Tạo Parameter object
p = Simulink.Parameter;
p.Value = params;
p.DataType = ['Bus: ', bus_name];
p.CoderInfo.StorageClass = 'ExportedGlobal'; % Cho phép dùng global

% Gán vào Model Workspace
assignin(hws, 'params', p);

fprintf('✅ Đã tạo Parameter "params" trong Model Workspace với Bus: %s\n', bus_name);