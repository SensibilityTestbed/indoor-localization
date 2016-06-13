%% clear all data
clear all;
close all;




%% File loading
filename = 'kalman_roll_z_axis.csv'; % experiment file





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
    plot3([-x_x(i)/2, x_x(i)/2], [-x_y(i)/2, x_y(i)/2], [-x_z(i)/2, x_z(i)/2], 'k','LineWidth',1)
    plot3([-x_x(i)/2 + y_x(i), x_x(i)/2 + y_x(i)], [-x_y(i)/2 + y_y(i), x_y(i)/2 + y_y(i)], [-x_z(i)/2 + y_z(i), x_z(i)/2 +  y_z(i)], 'k','LineWidth',1)
    plot3([x_x(i)/2, x_x(i)/2 + y_x(i)], [x_y(i)/2, x_y(i)/2 + y_y(i)], [x_z(i)/2, x_z(i)/2 +  y_z(i)], 'k','LineWidth',1)
    plot3([-x_x(i)/2 + y_x(i), -x_x(i)/2], [-x_y(i)/2 + y_y(i),-x_y(i)/2], [-x_z(i)/2 + y_z(i),-x_z(i)/2], 'k','LineWidth',1)
    plot3([0, y_x(i)], [0, y_y(i)], [0, y_z(i)], 'r', 'LineWidth',3)
    plot3([0, z_x(i)], [0, z_y(i)], [0, z_z(i)], 'g', 'LineWidth',3)
    hold off
    grid on
    axis([-1 1 -1 1 -1 1])
    title('Device Orientation Tracking')
    xlabel('x-axis in global coordinate')
    ylabel('y-axis in global coordinate')
    zlabel('z-axis in global coordinate')
    pause(0.01)
end