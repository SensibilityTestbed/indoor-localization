clear all;
close all;

%% File loading and data processing
filename = 'sony_50hz_trousers2.csv';
Fs = 50 % sampling frequency
HEIGHT = 1.75

data = csvread(filename,1,0)
time = data(:,1) * 10^-9
ax = data(:,2)
ay = data(:,3)
az = data(:,4)
gx = data(:,5)
gy = data(:,6)
gz = data(:,7)
mx = data(:,8)
my = data(:,9)
mz = data(:,10)

mag = sqrt(ax.^2+ay.^2+az.^2);
magNoG = mag - mean(mag);

%% Filter

% Low pass filter
Flp=3; % low pass filter cutofff frequency 0.8HZ
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
LPFmag = filtfilt(b,a,magNoG)

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
filteredMagNoG = filter(b, a, LPFmag);

intergralFilteredGx = (0 + cumtrapz(time,LPFGx));
intergralFilteredGy = (0 + cumtrapz(time,LPFGy));
intergralFilteredGz = (0 + cumtrapz(time,LPFGz));

magGyro = sqrt(intergralFilteredGx.^2+intergralFilteredGy.^2+intergralFilteredGz.^2);

gyroX = mod(intergralFilteredGx, 2*pi) * 180/pi
gyroY = mod(intergralFilteredGy, 2*pi) * 180/pi
gyroZ = mod(intergralFilteredGz, 2*pi) * 180/pi

thetaGyro = mod(magGyro, 2*pi)







%% Zero Crossing Method

n = length(filteredMagNoG);
ind = 1:(n-1);

%minPeakHeight = std(filteredMagNoG);
minPeakHeight = 0.2

y = filteredMagNoG > minPeakHeight;

k = find((y(ind)<=0) & (y(ind+1)>0)) ;

xc = [];

L = (y(k)==0) & (y(k+1)==0);
if any(L) 
xc = time(k(L));
k(L)=[];
end

if ~isempty(k)
s = (y(k+1)-y(k))./(time(k+1)-time(k));
xc = [xc,time(k) - y(k)./s];
end

numStepsZ = length(xc)

%% Peak Method Without Filtering

%minPeakHeight = std(magNoG);
minPeakHeight = 0.1;
[pks, locs] = findpeaks(filteredMagNoG, 'MINPEAKHEIGHT', minPeakHeight);

numStepsP = numel(pks);


%% Position

stride = HEIGHT * 0.415;

positionX = [0];
positionY = [0];

for i = 2:numStepsZ
    lastx = positionX(length(positionX));
    lasty = positionY(length(positionY));
    theta = thetaGyro(k(i))
    dPositionX = stride*cos(theta)
    dPositionY = stride*sin(theta)
    positionX = [positionX lastx+dPositionX];
    positionY = [positionY lasty+dPositionY];
    
end


positionX1 = [0];
positionY1 = [0];

for i = 2:numStepsP
    lastx = positionX1(length(positionX1));
    lasty = positionY1(length(positionY1));
    theta = thetaGyro(locs(i))
    dPositionX = stride*cos(theta)
    dPositionY = stride*sin(theta)
    positionX1 = [positionX1 lastx+dPositionX];
    positionY1 = [positionY1 lasty+dPositionY];
    
end

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




figure
subplot(2,1,1)
plot(time, filteredMagNoG)
hold on
plot(xc, zeros(length(xc)), 'r', 'Marker', 'v', 'LineStyle', 'none');
strNum = num2str(numStepsZ)
title('Non-Gavity Magnitude of accleration over time with Zero Crossing')
xlabel('time/seconds')
ylabel('m/s^2')
legend('Original Non-Gravity Magnitude of Acceleration', ['Crossing Zero: ' strNum], 'location','northoutside')
hold off

subplot(2,1,2)
plot(time, filteredMagNoG)
hold on
%plot(time(zeroCrossingIndex), zeros, 'b', 'Marker', 'v', 'LineStyle', 'none');
%plot(time(zeroCrossingNIndex), zerosN, 'b', 'Marker', 'v', 'LineStyle', 'none');
plot(time(locs), pks, 'r', 'Marker', 'v', 'LineStyle', 'none');
strNum = num2str(numStepsP)
title('Non-Gavity Magnitude of accleration over time with Peak find')
xlabel('time/seconds')
ylabel('m/s^2')
legend('Original Non-Gravity Magnitude of Acceleration', ['Peaks: ' strNum], 'location','northoutside')
hold off
saveas(gcf,'figure5.png')


figure
plot(positionX, positionY)
hold on
plot(positionX, positionY, 'Marker', 'v', 'LineStyle', 'none');
plot(positionX1, positionY1, 'r')
plot(positionX1, positionY1, 'r', 'Marker', 'v', 'LineStyle', 'none');
title('Indoor path tracking')
xlabel('meter')
ylabel('meter')
legend('from zero crossing', 'from peak search')

hold off
saveas(gcf,'figure6.png')



