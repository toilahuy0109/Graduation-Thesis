function Trajectory(block)
% S-Function Level-2 để mô phỏng quỹ đạo 5 drone (x,y,z) trong đồ thị 3D

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
    figure;
    hold on;
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Quỹ đạo 5 Drone trong không gian 3D');
    view(3); % góc nhìn 3D
    axis auto; % tự động mở rộng theo dữ liệu, không giới hạn khung hình
    
    colors = lines(5);
    for i = 1:5
        h(i) = plot3(0,0,0,'Color',colors(i,:),'LineWidth',2);
    end
    setappdata(0,'DroneHandles',h);
    
% -----------------------------
function Outputs(block)
    h = getappdata(0,'DroneHandles');
    for i = 1:5
        pos = block.InputPort(i).Data; % [x y z]
        % Cập nhật dữ liệu vẽ
        xdata = get(h(i),'XData');
        ydata = get(h(i),'YData');
        zdata = get(h(i),'ZData');
        set(h(i),'XData',[xdata pos(1)], ...
                 'YData',[ydata pos(2)], ...
                 'ZData',[zdata pos(3)]);
    end
    drawnow;

% -----------------------------
function Terminate(block)
    disp('Kết thúc mô phỏng quỹ đạo drone.');