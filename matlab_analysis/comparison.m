clear all;
close all;

%% File loading and data processing
filename = 'sony_50hz_hand1.csv';
Fs = 100 % sampling frequency

data = csvread(filename,1,0)
time = data(:,1)
ax = data(:,2)
ay = data(:,3)
az = data(:,4)
gx = data(:,5)
gy = data(:,6)
gz = data(:,7)
mx = data(:,8)
my = data(:,9)
mz = data(:,10)


%% Filter

% Low pass filter
Flp=4; % low pass filter cutofff frequency 0.8HZ
[b,a]=butter(5,Flp*2/Fs,'low'); % Butter lowpass filter

LPFAx = filtfilt(b, a, ax)
LPFAy = filtfilt(b, a, ay)
LPFAz = filtfilt(b, a, az)
LPFMx = filtfilt(b, a, mx)
LPFMy = filtfilt(b, a, my)
LPFMz = filtfilt(b, a, mz)
LPFGx = filtfilt(b, a, gx)
LPFGy = filtfilt(b, a, gy)
LPFGz = filtfilt(b, a, gz)

a = 4;
b = ones(1, a)/a;

filteredAx = filtfilt(b, a, LPFAx);  %Moving average filter
filteredAy = filtfilt(b, a, LPFAy);
filteredAz = filtfilt(b, a, LPFAz);
filteredMx = filtfilt(b, a, LPFMx);
filteredMy = filtfilt(b, a, LPFMy);
filteredMz = filtfilt(b, a, LPFMz);
filteredGx = filter(b, a, LPFGx);
filteredGy = filter(b, a, LPFGy);
filteredGz = filter(b, a, LPFGz);

intergralFilteredGx = (0 + cumtrapz(time,gx))/10^9;
intergralFilteredGy = (0 + cumtrapz(time,gy))/10^9;
intergralFilteredGz = (0 + cumtrapz(time,gz))/10^9;

gyroX = mod(intergralFilteredGx, 2*pi) * 180/pi
gyroY = mod(intergralFilteredGy, 2*pi) * 180/pi
gyroZ = mod(intergralFilteredGz, 2*pi) * 180/pi



%% Figure
figure
subplot(2,1,1)
plot(time, ax)
hold on
plot(time,ay, 'r')
plot(time,az, 'g')
title('Acceleration Magnitude acceleration over time')
xlabel('time/seconds')
ylabel('m/s^2')
legend('X-axis', 'Y-axis', 'Z-axis' );
hold off

subplot(2,1,2)
plot(time, filteredAx)
hold on
plot(time,filteredAy, 'r')
plot(time,filteredAz, 'g')
title('Filtered Acceleration Magnitude acceleration over time')
xlabel('time/seconds')
ylabel('m/s^2')
legend('X-axis', 'Y-axis', 'Z-axis' );
hold off

saveas(gcf,'figure1.png')

figure
subplot(2,1,1)
plot(time, gx)
hold on
plot(time,gy, 'r')
plot(time,gz, 'g')
title('Gyro Magnitude over time')
xlabel('time/seconds')
ylabel('rad/s')
legend('X-axis', 'Y-axis', 'Z-axis' );
hold off

subplot(2,1,2)
plot(time, LPFGx)
hold on
plot(time,LPFGy, 'r')
plot(time,LPFGz, 'g')
title('Filtered Gyro Magnitude over time')
xlabel('time/seconds')
ylabel('rad/s')
legend('X-axis', 'Y-axis', 'Z-axis' );
hold off

saveas(gcf,'figure2.png')

figure
subplot(2,1,1)
plot(time, mx)
hold on
plot(time,my, 'r')
plot(time,mz, 'g')
title('Magnetic Magnitude over time')
xlabel('time/seconds')
ylabel('micro tesla')
legend('X-axis', 'Y-axis', 'Z-axis' );
hold off


subplot(2,1,2)
plot(time, filteredMx)
hold on
plot(time,filteredMy, 'r')
plot(time,filteredMz, 'g')
title('Filtered Magnetic Magnitude over time')
xlabel('time/seconds')
ylabel('micro tesla')
legend('X-axis', 'Y-axis', 'Z-axis' );
hold off

saveas(gcf,'figure3.png')


figure

subplot(2,1,1)
plot(time, intergralFilteredGx)
hold on
plot(time,intergralFilteredGy, 'r')
plot(time,intergralFilteredGz, 'g')
title('Intergraled Gyro Magnitude over time')
xlabel('time/seconds')
ylabel('?')
legend('X-axis', 'Y-axis', 'Z-axis' );
hold off

subplot(2,1,2)
plot(time, gyroX)
hold on
plot(time,gyroY, 'r')
plot(time,gyroZ, 'g')
title('Intergraled Gyro Magnitude over time')
xlabel('time/seconds')
ylabel('?')
legend('X-axis', 'Y-axis', 'Z-axis' );
hold off

saveas(gcf,'figure4.png')
