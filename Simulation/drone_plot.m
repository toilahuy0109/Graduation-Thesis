function drone_plot(block)
% S-function vẽ quỹ đạo 4 drone
% 4 đầu vào: p1, p2, p3, p4 (mỗi p là [x;y;z])

setup(block);

%endfunction

function setup(block)
    % Số đầu vào: 4
    block.NumInputPorts = 4;
    block.NumOutputPorts = 0;
    
    % Cấu hình đầu vào
    for i = 1:4
        block.InputPort(i).Dimensions = 3;      % [x;y;z]
        block.InputPort(i).DatatypeID = 0;      % double
        block.InputPort(i).Complexity = 'Real';
        block.InputPort(i).DirectFeedthrough = true;
        block.InputPort(i).SamplingMode = 'Sample';
    end
    
    % Thời gian lấy mẫu
    block.SampleTimes = [0.1 0];  % 0.1s lấy mẫu 1 lần
    
    % Đăng ký các phương thức
    block.RegBlockMethod('Outputs', @Output);
    block.RegBlockMethod('Terminate', @Terminate);
    
%endfunction

function Output(block)
    % Lấy dữ liệu đầu vào
    p1 = block.InputPort(1).Data;
    p2 = block.InputPort(2).Data;
    p3 = block.InputPort(3).Data;
    p4 = block.InputPort(4).Data;
    
    % ĐẢM BẢO DỮ LIỆU LÀ VECTOR HÀNG
    if size(p1,1) > size(p1,2)
        p1 = p1';  % chuyển thành vector hàng
    end
    if size(p2,1) > size(p2,2)
        p2 = p2';
    end
    if size(p3,1) > size(p3,2)
        p3 = p3';
    end
    if size(p4,1) > size(p4,2)
        p4 = p4';
    end
    
    % Tạo mảng dữ liệu - CÁCH 1: Ghép theo hàng
    current_data = [block.CurrentTime, p1, p2, p3, p4];
    
    % Lưu vào persistent variable
    persistent history
    if isempty(history)
        history = [];
    end
    
    % CÁCH 2: Kiểm tra kích thước trước khi ghép
    if isempty(history)
        history = current_data;
    else
        % Đảm bảo history và current_data có cùng số cột
        if size(history,2) == size(current_data,2)
            history = [history; current_data];
        else
            % Nếu không khớp, tạo history mới
            disp('Warning: Dimension mismatch, resetting history');
            history = current_data;
        end
    end
    
    % Giới hạn kích thước history (tránh đầy bộ nhớ)
    if size(history,1) > 1000
        history = history(end-999:end, :);
    end
    
    % Vẽ mỗi 10 lần (1 giây)
    persistent counter
    if isempty(counter)
        counter = 0;
    end
    counter = counter + 1;
    
    if mod(counter, 10) == 0
        plot_trajectories(history);
    end
    
%endfunction

function Terminate(~)
    % Đóng figure khi kết thúc
    close all;
    
%endfunction

function plot_trajectories(history)
    % Kiểm tra history có đủ dữ liệu không
    if size(history,1) < 2 || size(history,2) < 13
        return;
    end
    
    % Vẽ quỹ đạo
    figure(1);
    
    % Tách dữ liệu
    t = history(:,1);
    d1 = history(:,2:4);
    d2 = history(:,5:7);
    d3 = history(:,8:10);
    d4 = history(:,11:13);
    
    % Xóa figure cũ
    clf;
    
    % Vẽ 3D
    subplot(2,2,1);
    plot3(d1(:,1), d1(:,2), d1(:,3), 'r.-', 'LineWidth', 1); hold on;
    plot3(d2(:,1), d2(:,2), d2(:,3), 'b.-', 'LineWidth', 1);
    plot3(d3(:,1), d3(:,2), d3(:,3), 'g.-', 'LineWidth', 1);
    plot3(d4(:,1), d4(:,2), d4(:,3), 'm.-', 'LineWidth', 1);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(sprintf('3D Trajectories (t=%.1f)', t(end)));
    grid on; view(45,30); legend({'Drone1','Drone2','Drone3','Drone4'});
    hold off;
    
    % XY
    subplot(2,2,2);
    plot(d1(:,1), d1(:,2), 'r.-', 'LineWidth', 1); hold on;
    plot(d2(:,1), d2(:,2), 'b.-', 'LineWidth', 1);
    plot(d3(:,1), d3(:,2), 'g.-', 'LineWidth', 1);
    plot(d4(:,1), d4(:,2), 'm.-', 'LineWidth', 1);
    xlabel('X'); ylabel('Y');
    title('XY Projection');
    grid on; axis equal; hold off;
    
    % XZ
    subplot(2,2,3);
    plot(d1(:,1), d1(:,3), 'r.-', 'LineWidth', 1); hold on;
    plot(d2(:,1), d2(:,3), 'b.-', 'LineWidth', 1);
    plot(d3(:,1), d3(:,3), 'g.-', 'LineWidth', 1);
    plot(d4(:,1), d4(:,3), 'm.-', 'LineWidth', 1);
    xlabel('X'); ylabel('Z');
    title('XZ Projection');
    grid on; axis equal; hold off;
    
    % YZ
    subplot(2,2,4);
    plot(d1(:,2), d1(:,3), 'r.-', 'LineWidth', 1); hold on;
    plot(d2(:,2), d2(:,3), 'b.-', 'LineWidth', 1);
    plot(d3(:,2), d3(:,3), 'g.-', 'LineWidth', 1);
    plot(d4(:,2), d4(:,3), 'm.-', 'LineWidth', 1);
    xlabel('Y'); ylabel('Z');
    title('YZ Projection');
    grid on; axis equal; hold off;
    
    drawnow;
%endfunction