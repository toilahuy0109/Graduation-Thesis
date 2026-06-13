function formation_sfunc(block)
    setup(block);
end

function setup(block)
    block.NumInputPorts = 1;
    block.NumOutputPorts = 1;

    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    %% Input
    % [xc yc xt yt]
    %
    block.InputPort(1).Dimensions = 4;
    block.InputPort(1).DatatypeID = 0;
    block.InputPort(1).Complexity = 'Real';
    block.InputPort(1).DirectFeedthrough = true;

    %% Output
    % [x1 y1 x2 y2 x3 y3 x4 y4]
    %
    block.OutputPort(1).Dimensions = 8;
    block.OutputPort(1).DatatypeID = 0;
    
    block.SampleTimes = [0 0];
    block.SimStateCompliance = 'DefaultSimState';
    
    block.RegBlockMethod('Outputs', @Outputs);
end

%% Output function
function Outputs(block)
    u = block.InputPort(1).Data;

    xc = u(1);
    yc = u(2);

    xt = u(3);
    yt = u(4);

    formation = block.DialogPrm(1).Data;
    d = block.DialogPrm(2).Data;

    theta = atan2(yt - yc, xt - xc);

    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];

    switch formation
        %% 1 = Line
        case 1
            rel = [-1.5*d 0;
                    -0.5*d 0;
                    0.5*d 0;
                    1.5*d 0];
        %% 2 = Triangle
        case 2
            rel = [-d/2 0;
                    d/2 0;
                    0 sqrt(3)/2*d;
                    0 -sqrt(3)/2*d];

        %% 3 = Square
        case 3
            rel = [-d/2 -d/2;
                    d/2 -d/2;
                    d/2 d/2;
                    -d/2 d/2];

        %% 4 = Diamond
        case 4
            rel = [-d 0;
                    0 d;
                    d 0;
                    0 -d];
        
        otherwise
            rel = zeros(4,2);
    end

    out = zeros(8,1);
    
    for k = 1:4
        p = [xc; yc] + R*rel(k,:)';

        out(2*k-1) = p(1);
        out(2*k) = p(2);
    end
    block.OutputPort(1).Data = out;
end