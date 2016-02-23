clear all
close all

%% File loading and data processing
filename_raw = 'raw_2_5_13_43.csv';
filename_filtered = 'filter_2_5_13_43.csv';
filename_step = 'step_2_5_13_43.csv'

M_raw = csvread(filename_raw,1,0)
time = M_raw(:,1)
magNoG = M_raw(:,2)

M_filtered = csvread(filename_filtered,1,0)
magNoG_filtered = M_filtered(:,2)

s = csvread(filename_step,1,0)
timeS = s(:,1)
stepCount = s(:,2)
%% Figure
figure

plot(time,magNoG)
hold on
plot(time,magNoG_filtered, 'r')
plot(timeS,zeros(length(timeS)), 'g', 'Marker', 'v', 'LineStyle', 'none')
title('Non-Gavity Magnitude acceleration over time')
xlabel('time/seconds')
ylabel('m/s^2')
legend('raw data', 'filtered data', 'threshold crossing' );
hold off



sd = std(magNoG)
sd1 = std(magNoG_filtered)