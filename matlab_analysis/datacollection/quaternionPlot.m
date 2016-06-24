%% clear all data
clear all;
close all;




%% File loading
namestring = 'yu\quarternion_ref\yu_';
namestring1 = '_ref_q.csv';
string = 't1';
filename = 'yu\ref\yu_t1_ref.csv';

data = csvread(filename,1,0);
count = 1:length(data); % setup counter array
time = data(:,1) * 10^-9; % each sample's collect time
ax = data(:,2); % accelerometer x-axis
ay = data(:,3); % accelerometer y-axis
az = data(:,4); % accelerometer z-axis
gx = data(:,5); % gyroscope x-axis
gy = data(:,6); % gyroscope y-axis
gz = data(:,7); % gyroscope z-axis
mx = data(:,8); % magnetometer x-axis
my = data(:,9); % magnetometer y-axis
mz = data(:,10); % magnetometer z-axis

% Magnititude of acceleration data from experiment data
mag = sqrt(ax.^2+ay.^2+az.^2);

% Non-gravity of acceleration
magNoG = mag - mean(mag);

% Samping frequency of experiment
Fs = length(time) / (time(length(time)) - time(1));






%% Multi files plots
%{
for i = {'call1','hand1','t1','b1'}
    string = char(i);
    filename = [namestring string namestring1];
   

   
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
    title(string)
end
%}



%% Low pass filter Filter
% All experiment data and reference data go through the same
% lowpass fileter

% Low pass filter
Flp=3; % low pass filter cutofff frequency 0.8HZ
[b,a]=butter(5,Flp*2/Fs,'low'); % Butter lowpass filter setup

% Still failed to write these in functions
% filtfilt function to cutoff the initial bias of filter
LPFAx = filtfilt(b, a, ax); % LPF output of acceleration x-axis
LPFAy = filtfilt(b, a, ay);
LPFAz = filtfilt(b, a, az);
LPFmag = filtfilt(b,a,magNoG); % LPF output of non-g magnititude of acceleration




%% Moving average filter
% Experiment data also go throught the moving average filter
% Window size settup as 4
a = 4;
b = ones(1, a)/a;

% Moving average filtering
filteredAx = filtfilt(b, a, LPFAx);  %Moving average filter output for accelerometer x-aix
filteredAy = filtfilt(b, a, LPFAy);
filteredAz = filtfilt(b, a, LPFAz);
filteredMagNoG = filter(b, a, LPFmag);




%% Peak Method Without Filtering
% For peak searching, using Matlab peak searching function to get the peaks
% which are above the threshold(noise level), and decrease those peaks'
% time interval is less than the minimum steps interval.

%minPeakHeight = std(magNoG);
minPeakHeight = 0.1; % noise level threshold

[pkstemp, locstemp] = findpeaks(filteredMagNoG, 'MINPEAKHEIGHT', minPeakHeight); % pkstemp get the peak value, and locstemp get location

numStepsP = numel(pkstemp);
locs = [locstemp(1)];
pks = [pkstemp(1)];

% delete the points whose internal is less than minimum step interval
for i = 2:numStepsP
    if time(locstemp(i)) - time(locstemp(i-1)) >= 0.25
        locs = [locs, locstemp(i)];
        pks = [pks, pkstemp(i)];
    end
end
numStepsP = length(locs);



%% Timestamp with pedometer


filename_q = [namestring string namestring1];

data_q = csvread(filename_q,1,0);

% Read quaternion data
x = data_q(:,1); 
y = data_q(:,2);
z = data_q(:,3);
s = data_q(:,4);

for i = 1: length(x)
    sum_temp = x(i).^2 + y(i).^2 + z(i).^2;
    if sum_temp ~= 1.0
        x(i) = (1.0/sum_temp).^0.5 * x(i);
        y(i) = (1.0/sum_temp).^0.5 * y(i);
        z(i) = (1.0/sum_temp).^0.5 * z(i);
    end
end


%% pedometer and quaternion plot

figure
subplot(2,1,1)
plot(time, filteredMagNoG) % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
hold on
plot(time(locs), pks, 'g', 'Marker', 'v', 'LineStyle', 'none'); % Step detection point by peak-searching method

% plot between steps
plot(time(1:locs(1)), filteredMagNoG(1:locs(1)), 'r', 'LineWidth', 3)


strNum = num2str(numStepsP);
title('Non-Gavity Magnitude of accleration over time with Peak find detection')
xlabel('time/seconds')
ylabel('m/s^2')
legend('Non-Gravity Magnitude of Acceleration', ['Step counts by Peak Searching: ' strNum], 'location','northoutside')
hold off

% plot in 3D
subplot(2,1,2)
plot3(x,y,z)
hold on
plot3(x(1:locs(1)), y(1:locs(1)), z(1:locs(1)), 'r', 'LineWidth', 3)
hold off
grid on
title(string)
saveas(gcf,'figure0.png')

for i = 1:length(locs)-1
    
    figure
    subplot(2,1,1)
    plot(time, filteredMagNoG) % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
    hold on
    plot(time(locs), pks, 'g', 'Marker', 'v', 'LineStyle', 'none'); % Step detection point by peak-searching method

    % plot between steps
    plot(time(locs(i):locs(i+1)), filteredMagNoG(locs(i):locs(i+1)), 'r', 'LineWidth', 3)


    strNum = num2str(numStepsP);
    title('Non-Gavity Magnitude of accleration over time with Peak find detection')
    xlabel('time/seconds')
    ylabel('m/s^2')
    legend('Non-Gravity Magnitude of Acceleration', ['Step counts by Peak Searching: ' strNum], 'location','northoutside')
    hold off

    % plot in 3D
    subplot(2,1,2)
    plot3(x,y,z)
    hold on
    plot3(x(locs(i):locs(i+1)), y(locs(i):locs(i+1)), z(locs(i):locs(i+1)), 'r', 'LineWidth', 3)
    hold off
    grid on
    title(string)
    figname = sprintf('figure%d.png', i);
    saveas(gcf,figname)
end



figure
subplot(2,1,1)
plot(time, filteredMagNoG) % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
hold on
plot(time(locs), pks, 'g', 'Marker', 'v', 'LineStyle', 'none'); % Step detection point by peak-searching method

% plot between steps
plot(time(locs(length(locs)):length(time)), filteredMagNoG(locs(length(locs)):length(time)), 'r', 'LineWidth', 3)


strNum = num2str(numStepsP);
title('Non-Gavity Magnitude of accleration over time with Peak find detection')
xlabel('time/seconds')
ylabel('m/s^2')
legend('Non-Gravity Magnitude of Acceleration', ['Step counts by Peak Searching: ' strNum], 'location','northoutside')
hold off

% plot in 3D
subplot(2,1,2)
plot3(x,y,z)
hold on
plot3(x(locs(length(locs)):length(x)), y(locs(length(locs)):length(x)), z(locs(length(locs)):length(x)), 'r', 'LineWidth', 3)
hold off
grid on
title(string)

figname = sprintf('figure%d.png', i+1);
saveas(gcf,figname)




%% Animation plot
%{
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
%}