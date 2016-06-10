%% clear all data
clear all;
close all;




%% File loading
filename = 'kalman_q.csv'; % experiment file





%% Raw data processing
data = csvread(filename,1,0);

% Read quaternion data
x_x = data(:,1); 
x_y = data(:,2);
x_z = data(:,3);
y_x = data(:,4); 
y_y = data(:,5);
y_z = data(:,6);
z_x = data(:,7);
z_y = data(:,8); 
z_z = data(:,9);


% plot in 3D
figure
for i = 1:length(x_x)
    
    plot3([0, x_x(i)], [0, x_y(i)], [0, x_z(i)], 'LineWidth',3)
    hold on
    plot3([0, y_x(i)], [0, y_y(i)], [0, y_z(i)], 'r', 'LineWidth',3)
    plot3([0, z_x(i)], [0, z_y(i)], [0, z_z(i)], 'g', 'LineWidth',3)
    hold off
    grid on
    axis([-1 1 -1 1 -1 1])
    %legend('x-axis','y-axis','z-axis')
    pause
end