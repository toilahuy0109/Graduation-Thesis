function Trajectory(block)
% S-Function Level-2 để mô phỏng quỹ đạo 5 drone (x,y,z) trong đồ thị 3D
% Vẽ cả quỹ đạo và các cạnh nối giữa các drone theo ma trận edges

    setup(block);

% -----------------------------
function setup(block)

    % 5 input ports, mỗi port là vector [x y z]
    block.NumInputPorts  = 5;
    for i = 1:5
        block.InputPort(i).Dimensions        = 3;
        block.InputPort(i).DatatypeID        = 0;  % double
        block.InputPort(i).Complexity        = 'Real';
        block.InputPort(i).DirectFeedthrough = true;
    end

    % Không có output port
    block.NumOutputPorts = 0;

    % Sample time
    block.SampleTimes = [0 0]; % continuous

    % Đăng ký các phương thức
    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Outputs',              @Outputs);
    block.RegBlockMethod('Terminate',            @Terminate);

% -----------------------------
function InitConditions(block)
    % Khởi tạo figure 3D
    fig = figure('Name', 'Quỹ đạo 5 Drone', 'NumberTitle', 'off');
    clf(fig);
    hold on;
    grid on;
    xlabel('X (m)', 'FontSize', 12);
    ylabel('Y (m)', 'FontSize', 12);
    zlabel('Z (m)', 'FontSize', 12);
    title('Quỹ đạo 5 Drone trong không gian 3D', 'FontSize', 14);
    view(45, 30); % góc nhìn đẹp
    
    % Màu sắc cho từng drone
    colors = lines(5);
    
    % Tạo handles cho quỹ đạo
    traj_handles = zeros(1,5);
    for i = 1:5
        traj_handles(i) = plot3(0,0,0, 'Color', colors(i,:), 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Drone %d', i));
    end
    
    % Tạo handles cho điểm hiện tại
    point_handles = zeros(1,5);
    for i = 1:5
        point_handles(i) = plot3(0,0,0, 'o', 'Color', colors(i,:), ...
            'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
    end
    
    % Định nghĩa các cạnh nối (9 cạnh cho 5 drone)
    edges = [1 2; 1 3; 1 4; 1 5; 2 3; 2 4; 3 4; 3 5; 4 5];
    n_edges = size(edges, 1);
    
    % Tạo handles cho các cạnh
    edge_handles = zeros(1, n_edges);
    for e = 1:n_edges
        edge_handles(e) = plot3([0 0], [0 0], [0 0], 'g-', 'LineWidth', 1, ...
            'Color', [0.5 0.5 0.5]);
    end
    
    % Lưu handles vào appdata
    setappdata(0, 'TrajHandles', traj_handles);
    setappdata(0, 'PointHandles', point_handles);
    setappdata(0, 'EdgeHandles', edge_handles);
    setappdata(0, 'Edges', edges);
    setappdata(0, 'FirstCall', true);
    
    % Lưu vị trí trước đó để vẽ quỹ đạo
    pos_history = zeros(3, 5, 10000); % lưu tối đa 10000 điểm
    setappdata(0, 'PosHistory', pos_history);
    setappdata(0, 'HistoryCount', 0);
    
    legend('Location', 'best');
    axis auto;

% -----------------------------
% -----------------------------
function Outputs(block)
    % Lấy dữ liệu từ appdata
    traj_handles = getappdata(0, 'TrajHandles');
    point_handles = getappdata(0, 'PointHandles');
    edge_handles = getappdata(0, 'EdgeHandles');
    edges = getappdata(0, 'Edges');
    pos_history = getappdata(0, 'PosHistory');
    history_count = getappdata(0, 'HistoryCount');
    
    % Đọc vị trí hiện tại của 5 drone
    current_pos = zeros(3, 5);
    for i = 1:5
        current_pos(:,i) = block.InputPort(i).Data;
    end
    
    % Lưu vị trí vào lịch sử
    history_count = history_count + 1;
    pos_history(:, :, history_count) = current_pos;
    setappdata(0, 'PosHistory', pos_history);
    setappdata(0, 'HistoryCount', history_count);
    
    % Cập nhật quỹ đạo (vẽ từ đầu đến hiện tại)
    for i = 1:5
        % Lấy toàn bộ lịch sử của drone i
        xdata = squeeze(pos_history(1, i, 1:history_count));
        ydata = squeeze(pos_history(2, i, 1:history_count));
        zdata = squeeze(pos_history(3, i, 1:history_count));
        
        set(traj_handles(i), 'XData', xdata, 'YData', ydata, 'ZData', zdata);
        
        % Cập nhật điểm hiện tại
        set(point_handles(i), 'XData', current_pos(1,i), ...
                              'YData', current_pos(2,i), ...
                              'ZData', current_pos(3,i));
    end
    
    % Cập nhật các cạnh nối
    for e = 1:size(edges, 1)
        i = edges(e,1);
        j = edges(e,2);
        
        set(edge_handles(e), 'XData', [current_pos(1,i), current_pos(1,j)], ...
                              'YData', [current_pos(2,i), current_pos(2,j)], ...
                              'ZData', [current_pos(3,i), current_pos(3,j)]);
    end
    
    % Cập nhật tiêu đề với thời gian hiện tại
    current_time = block.CurrentTime;
    title(sprintf('Quỹ đạo 5 Drone - t = %.2f s', current_time));
    
    % === CẬP NHẬT KHUNG HÌNH ĐỘNG ===
    % Cách 1: Để MATLAB tự động điều chỉnh
    axis auto;
    
    % Cách 2: Hoặc dùng công thức tính toán (bỏ comment nếu muốn)
    % all_pos = reshape(pos_history(:, :, 1:history_count), 3, []);
    % if ~isempty(all_pos)
    %     xlim([min(all_pos(1,:))-3, max(all_pos(1,:))+3]);
    %     ylim([min(all_pos(2,:))-3, max(all_pos(2,:))+3]);
    %     zlim([min(all_pos(3,:))-3, max(all_pos(3,:))+3]);
    % end
    
    drawnow limitrate;

% -----------------------------
function Terminate(block)
    disp('Kết thúc mô phỏng quỹ đạo drone.');
    
    % Giải phóng appdata
    rmappdata(0, 'TrajHandles');
    rmappdata(0, 'PointHandles');
    rmappdata(0, 'EdgeHandles');
    rmappdata(0, 'Edges');
    rmappdata(0, 'FirstCall');
    rmappdata(0, 'PosHistory');
    rmappdata(0, 'HistoryCount');