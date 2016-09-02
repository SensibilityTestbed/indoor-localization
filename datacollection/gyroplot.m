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

gx = data(:,5);
gy = data(:,6);
gz = data(:,7);
% Magnititude of acceleration data from experiment data
mag_t = sqrt(gx.^2+gy.^2+gz.^2);

% Samping frequency of experiment
Fs_t = length(time_t) / (time_t(length(time_t)) - time_t(1));


% hand
data = csvread(filenameh,1,0);
count_h = 1:length(data); % setup counter array
time_h = data(:,1) * 10^-9; % each sample's collect time
gx = data(:,5); % accelerometer x-axis
gy = data(:,6); % accelerometer y-axis
gz = data(:,7); % accelerometer z-axis

% Magnititude of acceleration data from experiment data
mag_h = sqrt(gx.^2+gy.^2+gz.^2);

% Samping frequency of experiment
Fs_h = length(time_h) / (time_h(length(time_h)) - time_h(1));



% backpack
data = csvread(filenameb,1,0);
count_b = 1:length(data); % setup counter array
time_b = data(:,1) * 10^-9; % each sample's collect time
gx = data(:,5); % accelerometer x-axis
gy = data(:,6); % accelerometer y-axis
gz = data(:,7); % accelerometer z-axis

% Magnititude of acceleration data from experiment data
mag_b = sqrt(gx.^2+gy.^2+gz.^2);

% Samping frequency of experiment
Fs_b = length(time_b) / (time_b(length(time_b)) - time_b(1));






% call
data = csvread(filenamec,1,0);
count_c = 1:length(data); % setup counter array
time_c = data(:,1) * 10^-9; % each sample's collect time
gx = data(:,5); % accelerometer x-axis
gy = data(:,6); % accelerometer y-axis
gz = data(:,7); % accelerometer z-axis

% Magnititude of acceleration data from experiment data
mag_c = sqrt(gx.^2+gy.^2+gz.^2);

% Samping frequency of experiment
Fs_c = length(time_c) / (time_c(length(time_c)) - time_c(1));


%% Low pass filter Filter
% All experiment data and reference data go through the same
% lowpass fileter

% Low pass filter
Flp=3; % low pass filter cutofff frequency 0.8HZ
[b,a]=butter(5,Flp*2/Fs_t,'low'); % Butter lowpass filter setup

% Still failed to write these in functions
% filtfilt function to cutoff the initial bias of filter

LPFmag_t = filtfilt(b,a,mag_t); % LPF output of non-g magnititude of acceleration


[b,a]=butter(5,Flp*2/Fs_h,'low'); % Butter lowpass filter setup

% Still failed to write these in functions
% filtfilt function to cutoff the initial bias of filter

LPFmag_h = filtfilt(b,a,mag_h); % LPF output of non-g magnititude of acceleration


[b,a]=butter(5,Flp*2/Fs_b,'low'); % Butter lowpass filter setup

% Still failed to write these in functions
% filtfilt function to cutoff the initial bias of filter

LPFmag_b = filtfilt(b,a,mag_b); % LPF output of non-g magnititude of acceleration


[b,a]=butter(5,Flp*2/Fs_c,'low'); % Butter lowpass filter setup

% Still failed to write these in functions
% filtfilt function to cutoff the initial bias of filter

LPFmag_c = filtfilt(b,a,mag_c); % LPF output of non-g magnititude of acceleration


%% Pedometer plots

figure
subplot(2,2,1)
plot(count_t, LPFmag_t, 'k-') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
axis([1 500 -10 10])
title('pattern of trousers')
xlabel('time/seconds')
ylabel('m/s^2')

subplot(2,2,2)
plot(count_c, LPFmag_c, 'k-') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
axis([1 500 -10 10])
title('pattern of call')
xlabel('time/seconds')
ylabel('m/s^2')

subplot(2,2,3)
plot(count_h, LPFmag_h, 'k-') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
axis([1 500 -10 10])
title('pattern of hand')
xlabel('time/seconds')
ylabel('m/s^2')

subplot(2,2,4)
plot(count_b, LPFmag_b, 'k-') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data
axis([1 500 -2 8])
title('pattern of backpack')
xlabel('time/seconds')
ylabel('m/s^2')





figure
subplot(2,2,1)
plot(count_t, mag_t, 'k-') 

axis([1 500 -2 8])
title('pattern of trousers')
xlabel('samples/counts')
ylabel('angular velocity (rad/s)')

subplot(2,2,2)
plot(count_c, mag_c, 'k-') 

axis([1 500 -2 8])
title('pattern of call')
xlabel('samples/counts')
ylabel('angular velocity (rad/s)')

subplot(2,2,3)
plot(count_h, mag_h, 'k-')
axis([1 500 -2 8])
title('pattern of hand')
xlabel('samples/counts')
ylabel('angular velocity (rad/s)')

subplot(2,2,4)
plot(count_b, mag_b, 'k-') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data

axis([1 500 -2 8 ])
title('pattern of backpack')
xlabel('samples/counts')
ylabel('angular velocity (rad/s)')









figure
subplot(2,2,1)
plot(count_t(90:200)-89, LPFmag_t(90:200), 'k-') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data

axis([1 110 -1 5])
title('pattern of trousers')
xlabel('time/seconds')
ylabel('m/s^2')

subplot(2,2,2)
plot(count_c(131:237)-130, LPFmag_c(131:237), 'k-') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data

axis([1 110 -1.5 2])
title('pattern of call')
xlabel('time/seconds')
ylabel('m/s^2')

subplot(2,2,3)
plot(count_h(113:218)-112, LPFmag_h(113:218), 'k-') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data

axis([1 110 -1.5 2])
title('pattern of hand')
xlabel('time/seconds')
ylabel('m/s^2')

subplot(2,2,4)
plot(count_b(107:211)-106, LPFmag_b(107:211), 'k-') % Filtered(LPF+MAF) magnititude of non-gravity acceleration data

axis([1 110 -1.5 2.5])
title('pattern of backpack')
xlabel('time/seconds')
ylabel('m/s^2')

