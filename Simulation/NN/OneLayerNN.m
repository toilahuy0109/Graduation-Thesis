%% Kiểm tra hội tụ của mạng
clear; clc; close all;

%% Tạo dữ liệu mẫu
n_input = 5;
n_hidden = 20;
n_samples = 1000;

% Tạo hàm mục tiêu ngẫu nhiên
W_true = randn(n_input, n_input) * 0.5;
b_true = randn(n_input, 1) * 0.1;

% Sinh dữ liệu
X = randn(n_input, n_samples) * 2;  % Đầu vào
Y_true = W_true' * X + b_true;      % Đầu ra mục tiêu

%% Tạo và huấn luyện mạng
nn = OneLayerNN(n_input, n_hidden, 0.01);

fprintf('Huấn luyện mạng...\n');
losses = [];
n_epochs = 50;
batch_size = 32;

for epoch = 1:n_epochs
    epoch_loss = 0;
    n_batches = 0;
    
    % Shuffle dữ liệu
    idx = randperm(n_samples);
    X = X(:, idx);
    Y_true = Y_true(:, idx);
    
    for i = 1:batch_size:n_samples
        batch_end = min(i + batch_size - 1, n_samples);
        X_batch = X(:, i:batch_end);
        Y_batch = Y_true(:, i:batch_end);
        
        batch_loss = 0;
        for j = 1:size(X_batch, 2)
            loss = nn.update(X_batch(:,j), Y_batch(:,j));
            batch_loss = batch_loss + loss;
        end
        epoch_loss = epoch_loss + batch_loss / size(X_batch, 2);
        n_batches = n_batches + 1;
    end
    
    losses(epoch) = epoch_loss / n_batches;
    
    if mod(epoch, 10) == 0
        fprintf('Epoch %d, Loss: %.6f\n', epoch, losses(epoch));
    end
end

%% Vẽ loss
figure;
semilogy(losses, 'b-', 'LineWidth', 2);
xlabel('Epoch'); ylabel('Loss');
title('Hội tụ của mạng');
grid on;

%% Kiểm tra trên test set
X_test = randn(n_input, 100);
Y_test = W_true' * X_test + b_true;

errors = zeros(100, 1);
for i = 1:100
    y_pred = nn.forward(X_test(:,i));
    errors(i) = norm(y_pred - Y_test(:,i));
end

fprintf('\nKết quả test:\n');
fprintf('  Mean error: %.4f\n', mean(errors));
fprintf('  Max error: %.4f\n', max(errors));
fprintf('  Min error: %.4f\n', min(errors));