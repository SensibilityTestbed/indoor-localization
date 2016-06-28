%% clear all data
clear all;
close all;




%% File loading
filename = 'yanyan\raw\yanyan_t5.csv';

data = csvread(filename,1,0);
count = 1:length(data); % setup counter array
time = data(:,1) * 10^-9; % each sample's collect time
ax = data(:,2); % accelerometer x-axis
ay = data(:,3); % accelerometer y-axis
az = data(:,4); % accelerometer z-axis

% Magnititude of acceleration data from experiment data
mag = sqrt(ax.^2+ay.^2+az.^2);

% Non-gravity of acceleration
magNoG = mag - mean(mag);


figure
subplot(2,1,1)
plot(count, magNoG);
subplot(2,1,2)
plot(time, magNoG);