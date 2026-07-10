Robs = 5;

x = linspace(0,100,10000);

f_his = zeros(1, size(x,2));
df_his = zeros(1,size(x,2));

f = @(x) -exp(-x^2/Robs^2);
df = @(x) 2*x/Robs*exp(-x^2/Robs^2);

for i = 1:size(x,2)
    f_his(1,i) = f(x(1,i));
    df_his(1,i) = df(x(1,i));
end

figure('Name', 'Potential Function');
subplot(2,1,1);
plot(x, f_his);

subplot(2,1,2);
plot(x, df_his);