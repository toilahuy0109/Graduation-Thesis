function DataAnalysis(block)
    setup(block);
end

function setup(block)
    % 4 input ports
    block.NumInputPorts = 4;

    % Port 1: Number of agents (scalar)
    block.InputPort(1).Dimensions = 1;
    block.InputPort(1).DatatypeID = 0;
    block.InputPort(1).Complexity = 'Real';
    block.InputPort(1).DirectFeedthrough = true;

    % Port 2: Real Position (vector 3n x 1)
    block.InputPort(2).Dimensions = -1;
    block.InputPort(2).DatatypeID = 0;
    block.InputPort(2).Complexity = 'Real';
    block.InputPort(2).DirectFeedthrough = true;
    block.InputPort(2).DimensionsMode = 0;

    % Port 3: Estimate Position (vector 3n x 1)
    block.InputPort(3).Dimensions = -1;
    block.InputPort(3).DatatypeID = 0;
    block.InputPort(3).Complexity = 'Real';
    block.InputPort(3).DirectFeedthrough = true;
    block.InputPort(3).DimensionsMode = 0;

    % Port 4: Edges Matrix (N x 2)
    block.InputPort(4).Dimensions = [-1; -1];
    block.InputPort(4).DatatypeID = 0;
    block.InputPort(4).Complexity = 'Real';
    block.InputPort(4).DirectFeedthrough = true;
    block.InputPort(4).DimensionsMode = 0;

    block.NumOutputPorts = 0;
    block.SampleTimes = [0 0];

    % Method Register
    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Terminate', @Terminate);
    block.RegBlockMethod('SetInputPortDimensions', @SetInputDims);
    block.RegBlockMethod('SetInputPortDataType', @SetInputDataType);
end

% -----------------------------
function SetInputDims(block, port, dims_info)
    if port == 2
        if mod(dims_info, 3) ~= 0
            error('Input port 2 dimensions must be multiple of 3');
        end
        block.InputPort(2).Dimensions = dims_info;
        
    elseif port == 3
        if mod(dims_info, 3) ~= 0
            error('Input port 3 dimensions must be multiple of 3');
        end
        block.InputPort(3).Dimensions = dims_info;
        
    elseif port == 4
        if length(dims_info) == 2
            block.InputPort(4).Dimensions = dims_info;
        elseif length(dims_info) == 1
            n_edges = dims_info / 2;
            if mod(dims_info, 2) ~= 0
                error('Input port 4 dimensions must be even (N x 2)');
            end
            block.InputPort(4).Dimensions = [n_edges, 2];
        end
    end
end

% -----------------------------
function SetInputDataType(block, port, dtid)
    if port == 2 || port == 3 || port == 4
        block.InputPort(port).DatatypeID = 0;
    end
end

% -----------------------------
function InitConditions(block)
    % Xóa tất cả appdata cũ
    if isappdata(0, 'DataFigureCreated')
        rmappdata(0, 'DataFigureCreated');
    end
    if isappdata(0, 'DataFirstCall')
        rmappdata(0, 'DataFirstCall');
    end
    
    setappdata(0, 'DataFigureCreated', false);
    setappdata(0, 'DataFirstCall', true);
    setappdata(0, 'DataDisHistory', []);
    setappdata(0, 'DataPesErrHistory', []);
    setappdata(0, 'DataTimeHistory', []);
    setappdata(0, 'DataHistoryCount', 0);
    setappdata(0, 'DataEdges', []);
    setappdata(0, 'DataPesErrHandles', []);
    setappdata(0, 'DataDisHandles', []);
    setappdata(0, 'DataAxesHandles', []);
end

% -----------------------------
function Outputs(block)
    persistent first_pass_debug
    if isempty(first_pass_debug)
        first_pass_debug = 0;
    end
    
    % Read data from port 1
    n = round(block.InputPort(1).Data);
    if n < 1
        return;
    end
    
    % Read data from port 2
    pos_vec = block.InputPort(2).Data;
    if isempty(pos_vec) || length(pos_vec) ~= 3*n
        return;
    end
    pos_cur = reshape(pos_vec, 3, n);
    
    % Read data from port 3
    pes_vec = block.InputPort(3).Data;
    if isempty(pes_vec) || length(pes_vec) ~= 3*n
        return;
    end
    pes_cur = reshape(pes_vec, 3, n);
    
    % Read data from port 4
    edges_data = block.InputPort(4).Data;
    if isempty(edges_data)
        edges = [];
        n_edges = 0;
    else
        dims = block.InputPort(4).Dimensions;
        if length(dims) == 2
            edges = reshape(edges_data, dims(1), dims(2));
        else
            edges = edges_data;
        end
    end
    n_edges = size(edges, 1);
    
    %% Get handles from appdata
    figure_created = getappdata(0, 'DataFigureCreated');
    prev_n = getappdata(0, 'DataNV');
    first_call = getappdata(0, 'DataFirstCall');
    prev_edges = getappdata(0, 'DataEdges');
    
    %% Check changes and create figure if needed
    edges_changed = false;
    if ~isempty(edges) && ~isempty(prev_edges)
        if size(edges,1) ~= size(prev_edges,1) || ~isequal(edges, prev_edges)
            edges_changed = true;
        end
    elseif ~isempty(edges) || ~isempty(prev_edges)
        edges_changed = true;
    end
    
    if first_call || ~figure_created || n ~= prev_n || edges_changed
        % Tạo figure - ĐẢM BẢO HIỂN THỊ
        fig = figure('Name', 'Data Analysis - Error & Distance', ...
                     'NumberTitle', 'off', ...
                     'Position', [700, 100, 900, 700], ...
                     'Visible', 'on');  % Đảm bảo hiển thị
        clf(fig);
        
        % Subplot 1: Estimation Error
        ax1 = subplot(2,1,1);
        hold on;
        grid on;
        ylabel('Error (m)', 'FontSize', 12);
        xlabel('Time (s)', 'FontSize', 12);
        title('Estimation Error (Desired - Actual)', 'FontSize', 14);
        
        colors = lines(n);
        pes_err_handles = zeros(1, n);
        for i = 1:n
            pes_err_handles(i) = plot(ax1, NaN, NaN, 'Color', colors(i,:), ...
                'LineWidth', 1.5, 'DisplayName', sprintf('Drone %d', i));
        end
        legend(ax1, 'show', 'Location', 'best');
        
        % Subplot 2: Distance
        ax2 = subplot(2,1,2);
        hold on;
        grid on;
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Distance (m)', 'FontSize', 12);
        title('Distance between drones (Edges)', 'FontSize', 14);
        
        if n_edges > 0
            color_edges = lines(n_edges);
            dis_handles = zeros(1, n_edges);
            for e = 1:n_edges
                dis_handles(e) = plot(ax2, NaN, NaN, 'Color', color_edges(e,:), ...
                    'LineWidth', 1.5, 'DisplayName', sprintf('%d-%d', edges(e,1), edges(e,2)));
            end
            legend(ax2, 'show', 'Location', 'best');
        else
            dis_handles = [];
        end
        
        % Initialize history
        dis_history = zeros(n_edges, 10000);
        pes_err_history = zeros(n, 10000);
        time_history = zeros(1, 10000);
        history_count = 0;
        
        % Save to appdata
        setappdata(0, 'DataFigureCreated', true);
        setappdata(0, 'DataNV', n);
        setappdata(0, 'DataFirstCall', false);
        setappdata(0, 'DataDisHistory', dis_history);
        setappdata(0, 'DataPesErrHistory', pes_err_history);
        setappdata(0, 'DataTimeHistory', time_history);
        setappdata(0, 'DataHistoryCount', history_count);
        setappdata(0, 'DataEdges', edges);
        setappdata(0, 'DataPesErrHandles', pes_err_handles);
        setappdata(0, 'DataDisHandles', dis_handles);
        setappdata(0, 'DataAxesHandles', [ax1, ax2]);
        
        % In ra console để xác nhận
        fprintf('DataAnalysis: Figure created with %d drones, %d edges\n', n, n_edges);
        
        return;
    end
    
    %% Update Handles
    dis_history = getappdata(0, 'DataDisHistory');
    pes_err_history = getappdata(0, 'DataPesErrHistory');
    time_history = getappdata(0, 'DataTimeHistory');
    history_count = getappdata(0, 'DataHistoryCount');
    edges = getappdata(0, 'DataEdges');
    n_edges = size(edges, 1);
    pes_err_handles = getappdata(0, 'DataPesErrHandles');
    dis_handles = getappdata(0, 'DataDisHandles');
    ax_handles = getappdata(0, 'DataAxesHandles');
    
    % Kiểm tra nếu chưa có figure thì thoát
    if isempty(ax_handles)
        return;
    end
    
    ax1 = ax_handles(1);
    ax2 = ax_handles(2);
    
    % Save position to history
    current_time = block.CurrentTime;
    history_count = history_count + 1;
    
    % Mở rộng mảng nếu cần
    if history_count > size(dis_history, 2)
        new_dis = zeros(n_edges, size(dis_history, 2) + 5000);
        new_dis(:, 1:size(dis_history,2)) = dis_history;
        dis_history = new_dis;
        
        new_pes = zeros(n, size(pes_err_history, 2) + 5000);
        new_pes(:, 1:size(pes_err_history,2)) = pes_err_history;
        pes_err_history = new_pes;
        
        new_time = zeros(1, size(time_history, 2) + 5000);
        new_time(1, 1:size(time_history,2)) = time_history;
        time_history = new_time;
        
        setappdata(0, 'DataDisHistory', dis_history);
        setappdata(0, 'DataPesErrHistory', pes_err_history);
        setappdata(0, 'DataTimeHistory', time_history);
    end
    
    % Lưu dữ liệu
    time_history(1, history_count) = current_time;
    
    % Lưu khoảng cách
    if n_edges > 0
        for e = 1:n_edges
            i = edges(e,1);
            j = edges(e,2);
            if i <= n && j <= n
                dis_history(e, history_count) = norm(pos_cur(:,i) - pos_cur(:,j));
            end
        end
    end
    
    % Lưu lỗi ước lượng
    for i = 1:n
        pes_err_history(i, history_count) = norm(pes_cur(:,i) - pos_cur(:,i));
    end
    
    setappdata(0, 'DataDisHistory', dis_history);
    setappdata(0, 'DataPesErrHistory', pes_err_history);
    setappdata(0, 'DataTimeHistory', time_history);
    setappdata(0, 'DataHistoryCount', history_count);
    
    %% Plot - Estimation Error
    axes(ax1);
    for i = 1:n
        xdata = time_history(1, 1:history_count);
        ydata = pes_err_history(i, 1:history_count);
        set(pes_err_handles(i), 'XData', xdata, 'YData', ydata);
    end
    if history_count > 1
        set(ax1, 'XLim', [0, max(time_history(1,1:history_count))]);
    end
    drawnow;
    
    %% Plot - Distance
    if n_edges > 0 && ~isempty(dis_handles)
        axes(ax2);
        for e = 1:n_edges
            xdata = time_history(1, 1:history_count);
            ydata = dis_history(e, 1:history_count);
            set(dis_handles(e), 'XData', xdata, 'YData', ydata);
        end
        if history_count > 1
            set(ax2, 'XLim', [0, max(time_history(1,1:history_count))]);
        end
        drawnow;
    end
    
    % In ra console mỗi 100 lần để kiểm tra
    first_pass_debug = first_pass_debug + 1;
    if mod(first_pass_debug, 100) == 0
        fprintf('DataAnalysis: t=%.2f, history=%d, error max=%.3f\n', ...
                current_time, history_count, max(pes_err_history(:, history_count)));
    end
end

% -----------------------------
function Terminate(block)
    fprintf('DataAnalysis: Terminating...\n');
    
    % Kiểm tra tồn tại trước khi xóa
    if isappdata(0, 'DataFigureCreated')
        rmappdata(0, 'DataFigureCreated');
    end
    if isappdata(0, 'DataNV')
        rmappdata(0, 'DataNV');
    end
    if isappdata(0, 'DataFirstCall')
        rmappdata(0, 'DataFirstCall');
    end
    if isappdata(0, 'DataDisHistory')
        rmappdata(0, 'DataDisHistory');
    end
    if isappdata(0, 'DataPesErrHistory')
        rmappdata(0, 'DataPesErrHistory');
    end
    if isappdata(0, 'DataTimeHistory')
        rmappdata(0, 'DataTimeHistory');
    end
    if isappdata(0, 'DataHistoryCount')
        rmappdata(0, 'DataHistoryCount');
    end
    if isappdata(0, 'DataEdges')
        rmappdata(0, 'DataEdges');
    end
    if isappdata(0, 'DataPesErrHandles')
        rmappdata(0, 'DataPesErrHandles');
    end
    if isappdata(0, 'DataDisHandles')
        rmappdata(0, 'DataDisHandles');
    end
    if isappdata(0, 'DataAxesHandles')
        rmappdata(0, 'DataAxesHandles');
    end
    
    disp('Data Analysis Terminated');
end