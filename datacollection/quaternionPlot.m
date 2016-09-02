%% clear all data
clear all;
close all;




%% File loading
namestring = 'yu\quarternion_ref\yu_';
namestring1 = '_ref_q.csv';
string = 'hand1';
filenamet = 'yu\ref\yu_t2_ref.csv';
filenameh = 'yu\ref\yu_hand1_ref.csv';
filenameb = 'yu\ref\yu_b1_ref.csv';
filenamec = 'yu\ref\yu_call1_ref.csv';




% trousers
data = csvread(filenamet,1,0);
count_t = 1:length(data); % setup counter array
time_t = data(:,1) * 10^-9; % each sample's collect time
ax = data(:,2); % accelerometer x-axis
ay = data(:,3); % accelerometer y-axis
az = data(:,4); % accelerometer z-axis
gx = data(:,5);
gy = data(:,6);
gz = data(:,7);
% Magnititude of acceleration data from experiment data
mag = sqrt(ax.^2+ay.^2+az.^2);

% Non-gravity of acceleration
magNoG_t = mag - mean(mag);

% Samping frequency of experiment
Fs_t = length(time_t) / (time_t(length(time_t)) - time_t(1));


% hand
data = csvread(filenameh,1,0);
count_h = 1:length(data); % setup counter array
time_h = data(:,1) * 10^-9; % each sample's collect time
ax = data(:,2); % accelerometer x-axis
ay = data(:,3); % accelerometer y-axis
az = data(:,4); % accelerometer z-axis

% Magnititude of acceleration data from experiment data
mag = sqrt(ax.^2+ay.^2+az.^2);

% Non-gravity of acceleration
magNoG_h = mag - mean(mag);

% Samping frequency of experiment
Fs_h = length(time_h) / (time_h(length(time_h)) - time_h(1));



% backpack
data = csvread(filenameb,1,0);
count_b = 1:length(data); % setup counter array
time_b = data(:,1) * 10^-9; % each sample's collect time
ax = data(:,2); % accelerometer x-axis
ay = data(:,3); % accelerometer y-axis
az = data(:,4); % accelerometer z-axis

% Magnititude of acceleration data from experiment data
mag = sqrt(ax.^2+ay.^2+az.^2);

% Non-gravity of acceleration
magNoG_b = mag - mean(mag);

% Samping frequency of experiment
Fs_b = length(time_b) / (time_b(length(time_b)) - time_b(1));






% call
data = csvread(filenamec,1,0);
count_c = 1:length(data); % setup counter array
time_c = data(:,1) * 10^-9; % each sample's collect time
ax = data(:,2); % accelerometer x-axis
ay = data(:,3); % accelerometer y-axis
az = data(:,4); % accelerometer z-axis

% Magnititude of acceleration data from experiment data
mag = sqrt(ax.^2+ay.^2+az.^2);

% Non-gravity of acceleration
magNoG_c = mag - mean(mag);

% Samping frequency of experiment
Fs_c = length(time_c) / (time_c(length(time_c)) - time_c(1));




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
[b,a]=butter(5,Flp*2/Fs_t,'low'); % Butter lowpass filter setup

% Still failed to write these in functions
% filtfilt function to cutoff the initial bias of filter

LPFmag_t = filtfilt(b,a,magNoG_t); % LPF output of non-g magnititude of acceleration


[b,a]=butter(5,Flp*2/Fs_h,'low'); % Butter lowpass filter setup

% Still failed to write these in functions
% filtfilt function to cutoff the initial bias of filter

LPFmag_h = filtfilt(b,a,magNoG_h); % LPF output of non-g magnititude of acceleration


[b,a]=butter(5,Flp*2/Fs_b,'low'); % Butter lowpass filter setup

% Still failed to write these in functions
% filtfilt function to cutoff the initial bias of filter

LPFmag_b = filtfilt(b,a,magNoG_b); % LPF output of non-g magnititude of acceleration


[b,a]=butter(5,Flp*2/Fs_c,'low'); % Butter lowpass filter setup

% Still failed to write these in functions
% filtfilt function to cutoff the initial bias of filter

LPFmag_c = filtfilt(b,a,magNoG_c); % LPF output of non-g magnititude of acceleration

%% Moving average filter
% Experiment data also go throught the moving average filter
% Window size settup as 4
a = 4;
b = ones(1, a)/a;

% Moving average filtering

filteredMagNoG_t = filter(b, a, LPFmag_t);
filteredMagNoG_b = filter(b, a, LPFmag_b);
filteredMagNoG_c = filter(b, a, LPFmag_c);
filteredMagNoG_h = filter(b, a, LPFmag_h);



%% Peak Method Without Filtering
% For peak searching, using Matlab peak searching function to get the peaks
% which are above the threshold(noise level), and decrease those peaks'
% time interval is less than the minimum steps interval.

%minPeakHeight = std(magNoG);
minPeakHeight = 0.1; % noise level threshold

[pkstemp, locstemp] = findpeaks(filteredMagNoG_t, 'MINPEAKHEIGHT', minPeakHeight); % pkstemp get the peak value, and locstemp get location

numStepsP = numel(pkstemp);
locs_t = [locstemp(1)];
pks_t = [pkstemp(1)];

% delete the points whose internal is less than minimum step interval
for i = 2:numStepsP
    if time_t(locstemp(i)) - time_t(locstemp(i-1)) >= 0.25
        locs_t = [locs_t, locstemp(i)];
        pks_t = [pks_t, pkstemp(i)];
    end
end
numStepsP_t = length(locs_t);






%% Peak Method Without Filtering
% For peak searching, using Matlab peak searching function to get the peaks
% which are above the threshold(noise level), and decrease those peaks'
% time interval is less than the minimum steps interval.

%minPeakHeight = std(magNoG);
minPeakHeight = 0.1; % noise level threshold

[pkstemp, locstemp] = findpeaks(filteredMagNoG_c, 'MINPEAKHEIGHT', minPeakHeight); % pkstemp get the peak value, and locstemp get location

numStepsP = numel(pkstemp);
locs_c = [locstemp(1)];
pks_c = [pkstemp(1)];

% delete the points whose internal is less than minimum step interval
for i = 2:numStepsP
    if time_c(locstemp(i)) - time_c(locstemp(i-1)) >= 0.25
        locs_c = [locs_c, locstemp(i)];
        pks_c = [pks_c, pkstemp(i)];
    end
end
numStepsP_c = length(locs_c);







%% Peak Method Without Filtering
% For peak searching, using Matlab peak searching function to get the peaks
% which are above the threshold(noise level), and decrease those peaks'
% time interval is less than the minimum steps interval.

%minPeakHeight = std(magNoG);
minPeakHeight = 0.1; % noise level threshold

[pkstemp, locstemp] = findpeaks(filteredMagNoG_b, 'MINPEAKHEIGHT', minPeakHeight); % pkstemp get the peak value, and locstemp get location

numStepsP = numel(pkstemp);
locs_b = [locstemp(1)];
pks_b = [pkstemp(1)];

% delete the points whose internal is less than minimum step interval
for i = 2:numStepsP
    if time_b(locstemp(i)) - time_b(locstemp(i-1)) >= 0.25
        locs_b = [locs_b, locstemp(i)];
        pks_b = [pks_b, pkstemp(i)];
    end
end
numStepsP_b = length(locs_b);







%% Peak Method Without Filtering
% For peak searching, using Matlab peak searching function to get the peaks
% which are above the threshold(noise level), and decrease those peaks'
% time interval is less than the minimum steps interval.

%minPeakHeight = std(magNoG);
minPeakHeight = 0.1; % noise level threshold

[pkstemp, locstemp] = findpeaks(filteredMagNoG_h, 'MINPEAKHEIGHT', minPeakHeight); % pkstemp get the peak value, and locstemp get location

numStepsP = numel(pkstemp);
locs_h = [locstemp(1)];
pks_h = [pkstemp(1)];

% delete the points whose internal is less than minimum step interval
for i = 2:numStepsP
    if time_h(locstemp(i)) - time_h(locstemp(i-1)) >= 0.25
        locs_h = [locs_h, locstemp(i)];
        pks_h = [pks_h, pkstemp(i)];
    end
end
numStepsP_h = length(locs_h);





%% Timestamp with pedometer


filename_q = [namestring string namestring1];

data_q = csvread(filename_q,1,0);

% Read quaternion data
x = data_q(:,1); 
y = data_q(:,2);
z = data_q(:,3);
s = data_q(:,4);

mag_q = [];
% Normalization
for i = 1: length(x)
    sum_temp = x(i).^2 + y(i).^2 + z(i).^2;
    if sum_temp ~= 1.0
        x(i) = (1.0/sum_temp).^0.5 * x(i);
        y(i) = (1.0/sum_temp).^0.5 * y(i);
        z(i) = (1.0/sum_temp).^0.5 * z(i);
    end
end




%% Pedometer plots

figure
subplot(2,2,1)
plot(count_t, filteredMagNoG_t, 'k-') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
hold on
plot(count_t(locs_t), pks_t, 'r', 'Marker', 'v', 'LineStyle', 'none'); % Step detection point by peak-searching method
hold off
title('pattern of trousers')
xlabel('time/seconds')
ylabel('m/s^2')
legend('trousers', 'location','northoutside')

subplot(2,2,2)
plot(count_c, filteredMagNoG_c, 'b--') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
hold on
plot(count_c(locs_c), pks_c, 'k', 'Marker', 'o', 'LineStyle', 'none'); % Step detection point by peak-searching method
hold off
title('pattern of call')
xlabel('time/seconds')
ylabel('m/s^2')
legend('call', 'location','northoutside')

subplot(2,2,3)
plot(count_h, filteredMagNoG_h, 'r:') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
hold on
plot(count_h(locs_h), pks_h, 'b', 'Marker', '+', 'LineStyle', 'none'); % Step detection point by peak-searching method
hold off
title('pattern of hand')
xlabel('time/seconds')
ylabel('m/s^2')
legend('hand', 'location','northoutside')

subplot(2,2,4)
plot(count_b, filteredMagNoG_b, 'g-.') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
hold on
plot(count_b(locs_b), pks_b, 'y', 'Marker', '*', 'LineStyle', 'none'); % Step detection point by peak-searching method
hold off
title('pattern of backpack')
xlabel('time/seconds')
ylabel('m/s^2')
legend('backpack', 'location','northoutside')





figure
subplot(2,2,1)
plot(count_t, magNoG_t, 'k-') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
hold on
plot(count_t(locs_t), magNoG_t(locs_t), 'k', 'Marker', 'v', 'LineStyle', 'none'); % Step detection point by peak-searching method
hold off
axis([1 500 -10 10])
title('patter of trousers')
xlabel('samples/counts')
ylabel('acceleration (m/s^2)')

subplot(2,2,2)
plot(count_c, magNoG_c, 'k-') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
hold on
plot(count_c(locs_c), magNoG_c(locs_c), 'k', 'Marker', 'o', 'LineStyle', 'none'); % Step detection point by peak-searching method
hold off
axis([1 500 -10 10])
title('patter of call')
xlabel('samples/counts')
ylabel('acceleration (m/s^2)')

subplot(2,2,3)
plot(count_h, magNoG_h, 'k-') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
hold on
plot(count_h(locs_h), magNoG_h(locs_h), 'k', 'Marker', '+', 'LineStyle', 'none'); % Step detection point by peak-searching method
hold off
axis([1 500 -10 10])
title('patter of hand')
xlabel('samples/counts')
ylabel('acceleration (m/s^2)')

subplot(2,2,4)
plot(count_b, magNoG_b, 'k-') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
hold on
plot(count_b(locs_b), magNoG_b(locs_b), 'k', 'Marker', '*', 'LineStyle', 'none'); % Step detection point by peak-searching method
hold off
axis([1 500 -10 10])
title('patter of backpack')
xlabel('samples/counts')
ylabel('acceleration (m/s^2)')














figure
subplot(2,2,1)
plot(count_t(90:200)-89, filteredMagNoG_t(90:200), 'k-') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
hold on
plot(count_t(locs_t(4:7)-89), pks_t(4:7), 'k', 'Marker', 'v', 'LineStyle', 'none'); % Step detection point by peak-searching method
hold off
axis([1 110 -1.5 1.2])
title('pattern of trousers')
xlabel('time/seconds')
ylabel('m/s^2')
legend('trousers', 'location','northoutside')

subplot(2,2,2)
plot(count_c(131:237)-130, filteredMagNoG_c(131:237), 'k--') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
hold on
plot(count_c(locs_c(4:7)-130), pks_c(4:7), 'k', 'Marker', 'o', 'LineStyle', 'none'); % Step detection point by peak-searching method
hold off
axis([1 110 -1.5 1.2])
title('pattern of call')
xlabel('time/seconds')
ylabel('m/s^2')
legend('call', 'location','northoutside')

subplot(2,2,3)
plot(count_h(113:218)-112, filteredMagNoG_h(113:218), 'k:') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
hold on
plot(count_h(locs_h(4:7)-112), pks_h(4:7), 'k', 'Marker', '+', 'LineStyle', 'none'); % Step detection point by peak-searching method
hold off
axis([1 110 -1.5 1.2])
title('pattern of hand')
xlabel('time/seconds')
ylabel('m/s^2')
legend('hand', 'location','northoutside')

subplot(2,2,4)
plot(count_b(107:211)-106, filteredMagNoG_b(107:211), 'k-.') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
hold on
plot(count_b(locs_b(4:7)-106), pks_b(4:7), 'k', 'Marker', '*', 'LineStyle', 'none'); % Step detection point by peak-searching method
hold off
axis([1 110 -1.5 1.2])
title('pattern of backpack')
xlabel('time/seconds')
ylabel('m/s^2')
legend('backpack', 'location','northoutside')




figure
plot(count_t(90:200)-90, filteredMagNoG_t(90:200), 'k-') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
hold on
plot(count_t(locs_t(4:7)- 90), pks_t(4:7), 'k', 'Marker', 'v', 'LineStyle', 'none'); % Step detection point by peak-searching method
plot(count_c(131:237)-131, filteredMagNoG_c(131:237), 'k--') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
plot(count_c(locs_c(4:7)-131), pks_c(4:7), 'k', 'Marker', 'o', 'LineStyle', 'none'); % Step detection point by peak-searching method
plot(count_h(113:218)-113, filteredMagNoG_h(113:218), 'k:') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
plot(count_h(locs_h(4:7)-113), pks_h(4:7), 'k', 'Marker', '+', 'LineStyle', 'none'); % Step detection point by peak-searching method
plot(count_b(107:211)-107, filteredMagNoG_b(107:211), 'k-.') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
plot(count_b(locs_b(4:7)-107), pks_b(4:7), 'k', 'Marker', '*', 'LineStyle', 'none'); % Step detection point by peak-searching method
hold off
axis([1 120 -1.5 1.2])
title('pattern of 4 different carrying modes')
xlabel('time/seconds')
ylabel('m/s^2')
legend('trousers','trousers step count','call','call step count','hand','hand step count','backpack','backpack step count', 'location','northoutside')



figure
subplot(2,1,1)
plot(count_t(90:200)-90, filteredMagNoG_t(90:200), 'k-') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
hold on
plot(count_t(locs_t(4:7)- 90), pks_t(4:7), 'k', 'Marker', 'v', 'LineStyle', 'none'); % Step detection point by peak-searching method
plot(count_c(131:237)-131, filteredMagNoG_c(131:237), 'k--') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
plot(count_c(locs_c(4:7)-131), pks_c(4:7), 'k', 'Marker', 'o', 'LineStyle', 'none'); % Step detection point by peak-searching method
hold off
axis([1 120 -1.5 1.2])
title('pattern of 2 different carrying modes')
xlabel('time/seconds')
ylabel('m/s^2')
legend('trousers','trousers step count','call','call step count','location','northoutside')


subplot(2,1,2)
plot(count_h(113:218)-113, filteredMagNoG_h(113:218), 'k:') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
hold on
plot(count_h(locs_h(4:7)-113), pks_h(4:7), 'k', 'Marker', '+', 'LineStyle', 'none'); % Step detection point by peak-searching method
plot(count_b(107:211)-107, filteredMagNoG_b(107:211), 'k-.') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
plot(count_b(locs_b(4:7)-107), pks_b(4:7), 'k', 'Marker', '*', 'LineStyle', 'none'); % Step detection point by peak-searching method
hold off
axis([1 120 -1.5 1.2])
title('pattern of 2 different carrying modes')
xlabel('time/seconds')
ylabel('m/s^2')
legend('hand','hand step count','backpack','backpack step count', 'location','northoutside')





%% pedometer and quaternion plot

figure
subplot(2,1,1)
plot(time_h, filteredMagNoG_h,'k-') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
hold on
plot(time_h(locs_h), pks_h, 'k', 'Marker', 'v', 'LineStyle', 'none'); % Step detection point by peak-searching method

title('patter of hand')
xlabel('time/seconds')
ylabel('m/s^2')
hold off

% plot in 3D
subplot(2,1,2)
plot3(x,y,z,'k-')
grid on
title('quatenion')

meanX = mean(x(1:locs(1)));
meanY = mean(y(1:locs(1)));
meanZ = mean(z(1:locs(1)));

Sx = std(x(1:locs(1)));
Sy = std(y(1:locs(1)));
Sz = std(z(1:locs(1)));

StringX = sprintf('std x: %d, mean x: %d', Sx, meanX);
StringY = sprintf('std y: %d, mean y: %d', Sy, meanY);
StringZ = sprintf('std z: %d, mean z: %d', Sz, meanZ);

descr = {StringX;
    StringY;
    StringZ
    };


text(-1.9,0.5, -1.9,descr)



saveas(gcf,'figure0.png')

%{

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
    
    meanX = mean(x(locs(i):locs(i+1)));
    meanY = mean(y(locs(i):locs(i+1)));
    meanZ = mean(z(locs(i):locs(i+1)));
    
    Sx = std(x(locs(i):locs(i+1)));
    Sy = std(y(locs(i):locs(i+1)));
    Sz = std(z(locs(i):locs(i+1)));
    
    StringX = sprintf('std x: %d, mean x: %d', Sx, meanX);
    StringY = sprintf('std y: %d, mean y: %d', Sy, meanY);
    StringZ = sprintf('std z: %d, mean z: %d', Sz, meanZ);
    
    descr = {StringX;
        StringY;
        StringZ
        };

    
    text(-1.9,0.5, -1.9,descr)
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


meanX = mean(x(locs(length(locs)):length(x)));
meanY = mean(y(locs(length(locs)):length(x)));
meanZ = mean(z(locs(length(locs)):length(x)));

Sx = std(x(locs(length(locs)):length(x)));
Sy = std(y(locs(length(locs)):length(x)));
Sz = std(z(locs(length(locs)):length(x)));

StringX = sprintf('std x: %d, mean x: %d', Sx, meanX);
StringY = sprintf('std y: %d, mean y: %d', Sy, meanY);
StringZ = sprintf('std z: %d, mean z: %d', Sz, meanZ);

descr = {StringX;
    StringY;
    StringZ
    };


text(-1.9,0.5, -1.9,descr)




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




%}