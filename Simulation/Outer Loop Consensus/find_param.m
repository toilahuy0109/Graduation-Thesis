%% PARAMETER SWEEP
clear; clc;

K1_values = [2, 5, 10];
K2_values = [20, 30, 50];
K3_values = [0.3, 0.6, 1.0];
mu_values = [0.5, 1.0, 1.5, 2.0];

results = [];

for K1 = K1_values
    for K2 = K2_values
        for K3 = K3_values
            for mu = mu_values
                fprintf('\n=== Testing K1=%.2f, K2=%.2f, K3=%.2f, mu=%.2f ===\n', K1, K2, K3, mu);

                stats = consensus_attack(K1, K2, K3, mu);

                results = [results; K1, K2, K3, mu, ...
                           stats.final_formation_error, ...
                           stats.final_tracking_error, ...
                           stats.final_control_norm];
            end
        end
    end
end

disp(array2table(results, ...
    'VariableNames', {'K1','K2','K3','mu','FormationError','TrackingError','ControlNorm'}));

good_idx = results(:,5) < 0.5;
disp('=== Good parameter sets (FormationError < 0.5) ===');
disp(results(good_idx,:));