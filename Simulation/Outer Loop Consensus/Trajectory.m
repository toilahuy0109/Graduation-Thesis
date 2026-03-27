function Trajectory(block)
% S-Function Level-2 để mô phỏng quỹ đạo n phương tiện (x,y,z) trong không gian 3D
% Đầu vào 1: n (số lượng phương tiện) - hằng số
% Đầu vào 2: vector [3n x 1] chứa vị trí của tất cả phương tiện
% Đầu vào 3: ma trận edges [N x 2] chứa các cạnh kết nối

    setup(block);

% -----------------------------
function setup(block)
    % 3 input ports
    block.NumInputPorts = 3;
    
    % Port 1: số lượng phương tiện (scalar)
    block.InputPort(1).Dimensions = 1;
    block.InputPort(1).DatatypeID = 0;
    block.InputPort(1).Complexity = 'Real';
    block.InputPort(1).DirectFeedthrough = true;
    
    % Port 2: vị trí tất cả phương tiện (vector 3n x 1)
    block.InputPort(2).Dimensions = -1;  % dynamic
    block.InputPort(2).DatatypeID = 0;
    block.InputPort(2).Complexity = 'Real';
    block.InputPort(2).DirectFeedthrough = true;
    
    % Port 3: ma trận edges (N x 2)
    block.InputPort(3).Dimensions = [-1; -1];  % dynamic
    block.InputPort(3).DatatypeID = 0;
    block.InputPort(3).Complexity = 'Real';
    block.InputPort(3).DirectFeedthrough = true;
    
    % Không có output port
    block.NumOutputPorts = 0;
    
    % Sample time
    block.SampleTimes = [0 0];
    
    % Đăng ký các phương thức
    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Terminate', @Terminate);
    block.RegBlockMethod('SetInputPortDimensions', @SetInputDims);
    block.RegBlockMethod('SetInputPortDataType', @SetInputDataType);

% -----------------------------
function SetInputDims(block, port, dims_info)
    % Cho phép kích thước đầu vào thay đổi
    if port == 2
        block.InputPort(2).Dimensions = dims_info;
    elseif port == 3
        block.InputPort(3).Dimensions = dims_info;
    end

% -----------------------------
function SetInputDataType(block, port, dtid)
    % Đảm bảo kiểu dữ liệu là double
    if port == 2 || port == 3
        block.InputPort(port).DatatypeID = 0;
    end

% -----------------------------
function InitConditions(block)
    setappdata(0, 'FigureCreated', false);
    setappdata(0, 'n_vehicles', 0);
    setappdata(0, 'FirstCall', true);
    setappdata(0, 'PosHistory', []);
    setappdata(0, 'HistoryCount', 0);
    setappdata(0, 'TrajHandles', []);
    setappdata(0, 'PointHandles', []);
    setappdata(0, 'EdgeHandles', []);
    setappdata(0, 'Edges', []);

% -----------------------------
function Outputs(block)
    % =====================================================================
    %  ĐỌC ĐẦU VÀO
    % =====================================================================
    % Số lượng phương tiện
    n = round(block.InputPort(1).Data);
    if n < 1
        return;
    end
    
    % Vị trí (vector 3n x 1)
    pos_vec = block.InputPort(2).Data;
    if isempty(pos_vec) || length(pos_vec) ~= 3*n
        return;
    end
    current_pos = reshape(pos_vec, 3, n);
    
    % Ma trận edges từ input port 3
    edges_data = block.InputPort(3).Data;
    if isempty(edges_data)
        % Nếu không có edges, tạo đồ thị đầy đủ
        edges = [];
        for i = 1:n
            for j = i+1:n
                edges = [edges; i, j];
            end
        end
    else
        % Xác định kích thước của edges
        dims = block.InputPort(3).Dimensions;
        if length(dims) == 2
            edges = reshape(edges_data, dims(1), dims(2));
        else
            edges = edges_data;
        end
    end
    n_edges = size(edges, 1);
    
    % =====================================================================
    %  LẤY HANDLES TỪ APPDATA
    % =====================================================================
    traj_handles = getappdata(0, 'TrajHandles');
    point_handles = getappdata(0, 'PointHandles');
    edge_handles = getappdata(0, 'EdgeHandles');
    prev_edges = getappdata(0, 'Edges');
    pos_history = getappdata(0, 'PosHistory');
    history_count = getappdata(0, 'HistoryCount');
    first_call = getappdata(0, 'FirstCall');
    prev_n = getappdata(0, 'n_vehicles');
    figure_created = getappdata(0, 'FigureCreated');
    
    % =====================================================================
    %  KIỂM TRA THAY ĐỔI VÀ TẠO FIGURE NẾU CẦN
    % =====================================================================
    edges_changed = false;
    if size(edges,1) ~= size(prev_edges,1) || ~isequal(edges, prev_edges)
        edges_changed = true;
    end
    
    if n ~= prev_n || edges_changed || first_call || ~figure_created
        % Tạo figure mới
        close all;
        fig = figure('Name', 'Quỹ đạo các phương tiện', 'NumberTitle', 'off');
        clf(fig);
        hold on;
        grid on;
        xlabel('X (m)', 'FontSize', 12);
        ylabel('Y (m)', 'FontSize', 12);
        zlabel('Z (m)', 'FontSize', 12);
        title('Quỹ đạo các phương tiện trong không gian 3D', 'FontSize', 14);
        view(45, 30);
        
        % Màu sắc
        colors = lines(n);
        
        % Tạo handles cho quỹ đạo
        traj_handles = zeros(1, n);
        for i = 1:n
            traj_handles(i) = plot3(0, 0, 0, 'Color', colors(i,:), 'LineWidth', 1.5, ...
                'DisplayName', sprintf('Drone %d', i));
        end
        
        % Tạo handles cho điểm hiện tại
        point_handles = zeros(1, n);
        for i = 1:n
            point_handles(i) = plot3(0, 0, 0, 'o', 'Color', colors(i,:), ...
                'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
        end
        
        % Tạo handles cho các cạnh
        if n_edges > 0
            edge_handles = zeros(1, n_edges);
            for e = 1:n_edges
                edge_handles(e) = plot3([0 0], [0 0], [0 0], 'k-', 'LineWidth', 0.8, ...
                    'Color', [0.5 0.5 0.5]);
            end
        else
            edge_handles = [];
        end
        
        % Khởi tạo lịch sử
        pos_history = zeros(3, n, 10000);
        history_count = 0;
        
        % Lưu vào appdata
        setappdata(0, 'n_vehicles', n);
        setappdata(0, 'TrajHandles', traj_handles);
        setappdata(0, 'PointHandles', point_handles);
        setappdata(0, 'EdgeHandles', edge_handles);
        setappdata(0, 'Edges', edges);
        setappdata(0, 'PosHistory', pos_history);
        setappdata(0, 'HistoryCount', history_count);
        setappdata(0, 'FirstCall', false);
        setappdata(0, 'FigureCreated', true);
        
        figure_created = true;
    end
    
    % =====================================================================
    %  CẬP NHẬT LẠI HANDLES
    % =====================================================================
    traj_handles = getappdata(0, 'TrajHandles');
    point_handles = getappdata(0, 'PointHandles');
    edge_handles = getappdata(0, 'EdgeHandles');
    edges = getappdata(0, 'Edges');
    n_edges = size(edges, 1);
    pos_history = getappdata(0, 'PosHistory');
    history_count = getappdata(0, 'HistoryCount');
    
    % =====================================================================
    %  LƯU VỊ TRÍ VÀO LỊCH SỬ
    % =====================================================================
    history_count = history_count + 1;
    if history_count <= size(pos_history, 3)
        pos_history(:, :, history_count) = current_pos;
    else
        new_history = zeros(3, n, size(pos_history, 3) + 5000);
        new_history(:, :, 1:size(pos_history,3)) = pos_history;
        pos_history = new_history;
        pos_history(:, :, history_count) = current_pos;
    end
    setappdata(0, 'PosHistory', pos_history);
    setappdata(0, 'HistoryCount', history_count);
    
    % =====================================================================
    %  VẼ QUỸ ĐẠO
    % =====================================================================
    for i = 1:n
        xdata = squeeze(pos_history(1, i, 1:history_count));
        ydata = squeeze(pos_history(2, i, 1:history_count));
        zdata = squeeze(pos_history(3, i, 1:history_count));
        
        set(traj_handles(i), 'XData', xdata, 'YData', ydata, 'ZData', zdata);
        set(point_handles(i), 'XData', current_pos(1,i), ...
                              'YData', current_pos(2,i), ...
                              'ZData', current_pos(3,i));
    end
    
    % =====================================================================
    %  VẼ CÁC CẠNH
    % =====================================================================
    for e = 1:n_edges
        i = edges(e,1);
        j = edges(e,2);
        if i >= 1 && i <= n && j >= 1 && j <= n
            set(edge_handles(e), 'XData', [current_pos(1,i), current_pos(1,j)], ...
                                  'YData', [current_pos(2,i), current_pos(2,j)], ...
                                  'ZData', [current_pos(3,i), current_pos(3,j)]);
        end
    end
    
    % Ẩn cạnh thừa
    for e = n_edges+1:length(edge_handles)
        set(edge_handles(e), 'XData', [], 'YData', [], 'ZData', []);
    end
    
    % =====================================================================
    %  CẬP NHẬT KHUNG HÌNH
    % =====================================================================
    current_time = block.CurrentTime;
    title(sprintf('Quỹ đạo %d phương tiện - t = %.2f s', n, current_time));
    axis auto;
    drawnow limitrate;

% -----------------------------
function Terminate(block)
    disp('Kết thúc mô phỏng quỹ đạo.');
    rmappdata(0, 'FigureCreated');
    rmappdata(0, 'n_vehicles');
    rmappdata(0, 'TrajHandles');
    rmappdata(0, 'PointHandles');
    rmappdata(0, 'EdgeHandles');
    rmappdata(0, 'Edges');
    rmappdata(0, 'FirstCall');
    rmappdata(0, 'PosHistory');
    rmappdata(0, 'HistoryCount');