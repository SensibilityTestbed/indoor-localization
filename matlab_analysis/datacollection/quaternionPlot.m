%% clear all data
clear all;
close all;




%% File loading
filename = 'yu\quarternion_ref\yu_call1_ref_q.csv'; % experiment file





%% Raw data processing
data = csvread(filename,1,0);

% Read quaternion data
x = data(:,1); 
y = data(:,2);
z = data(:,3);
s = data(:,4);

% plot in 3D
figure
plot3(x,y,z)
grid on


figure

for i = 1:length(x)
    sum_temp = x(i).^2 + y(i).^2 + z(i).^2;
    if sum_temp ~= 1.0
        x(i) = (1.0/sum_temp).^0.5 * x(i);
        y(i) = (1.0/sum_temp).^0.5 * y(i);
        z(i) = (1.0/sum_temp).^0.5 * z(i);
    end
 
    scatter3(x(i), y(i), z(i), 'b')
    hold on
    grid on

    pause(0.01)
end
hold off
