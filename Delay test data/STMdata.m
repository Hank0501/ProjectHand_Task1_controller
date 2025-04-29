close all;


data = importdata('./STM/delay_tst0805.txt');

figure;
% plot(data.data(:,2))
plot(data.data(:,1),data.data(:,2));


figure;
start_index=150;
interval=150;

num_elements = floor((length(data.data(:,1)) - start_index) / interval) + 1;
b = zeros(1, num_elements);
c = zeros(1, num_elements);

for i = 1:num_elements
    index = start_index + (i - 1) * interval;
    c(i)=data.data(index,2);
end


plot(c);

