d_safe = 3;
muy = (1 + d_safe^4)/d_safe^4;

d = linspace(0, 10, 1000);

beta = zeros(1, 1000);

for i = 1:1000
    beta(1, i) = beta_slide(d(i), d_safe, muy);
end


function beta = beta_slide(dist, d_safe, muy)
    
    if (dist^2 - d_safe^2) >= 0
        rho = 0;
    else
        rho = 1;
    end
    diff_sq = (dist^2 - d_safe^2)^2;
    beta = (1 - muy * diff_sq / (1 + diff_sq))^rho;

    beta = max(beta, 0);
end


plot(d, beta, 'Color', 'red', 'LineWidth', 1);
grid on;
hold on;
xlim([-0.5 10]);
ylim([-0.2 1.4]);

xlabel('Distance (m)', 'FontSize', 13);
ylabel('$\beta_{ij}(\mathbf{p}(t))$', 'Interpreter', 'latex', 'FontSize', 14);

% Sau khi vẽ xong
annotation('textbox',[0.75 0.8 0.1 0.1], ... % vị trí theo normalized figure
           'String','$d_{safe} = 3 (m) \\ \alpha = 2 $', ...
           'Interpreter','latex', ...
           'FontSize',12, ...
           'EdgeColor','black', ...
           'BackgroundColor','white');