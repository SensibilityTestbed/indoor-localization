clear all
close all

%% File loading and data processing
filename = 'Hand_1_28_21_39_matlab.csv';
M = csvread(filename,1,0)
time = M(:,1)
X = M(:,2)
Y = M(:,3)
Z = M(:,4)
mag = sqrt(X.^2+Y.^2+Z.^2);
%magNoG = mag - 9.8;
magNoG = mag - mean(mag) %10.7233 for 4th

%% Figure
figure
plot(time,X)
title('X axis of acceleration over time')
xlabel('time/second')
ylabel('m/s^2')
saveas(gcf,'figure1.png')

figure
plot(time,Y)
title('Y axis of acceleration over time')
xlabel('time/seconds')
ylabel('m/s^2')
saveas(gcf,'figure2.png')

figure
plot(time,Z)
title('Z axis of acceleration over time')
xlabel('time/seconds')
ylabel('m/s^2')
saveas(gcf,'figure3.png')

figure
plot(time,X, time, Y, time, Z)
title('3D acceleration over time')
xlabel('time/seconds)')
ylabel('m/s^2')
legend('X', 'Y', 'Z')
saveas(gcf,'figure4.png')

figure
plot(time,magNoG)
title('Non-Gavity Magnitude acceleration over time')
xlabel('time/seconds')
ylabel('m/s^2')

%% Low Pass Filter
%B = [1 -0.97]; % 0.95 to 0.97 pre-emphasis filter
%filteredMagNoG = filter(B,1,magNoG);


Fs=14; % sampling frequency
Flp=0.8; % low pass filter cutofff frequency 0.8HZ
[b,a]=butter(5,Flp/Fs,'low'); % Butter lowpass filter
filteredMagNoG = filter(b,a,magNoG); 


%{
samplingPerStep = 20;
coeff = ones(1, samplingPerStep)/samplingPerStep;
filteredMagNoG = filter(coeff, 1, filteredMagNoG);  %Moving average filter
%}

%{
h = [1/2 1/2];
binomialCoeff = conv(h,h);
for n = 1:4
    binomialCoeff = conv(binomialCoeff,h);
end
filteredMagNoG = filter(binomialCoeff, 1, magNoG);  %Binomial Filter
%}



hold on
plot(time, filteredMagNoG, 'r')
legend('Without filter','Filtered')
hold off
saveas(gcf,'figure5.png')

%% Zero Crossing Method

n = length(filteredMagNoG);
ind = 1:(n-1);

minPeakHeight = std(filteredMagNoG);
y = filteredMagNoG
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

numSteps = length(xc)

figure
plot(time, filteredMagNoG)
hold on
plot(xc, zeros(length(xc)), 'r', 'Marker', 'v', 'LineStyle', 'none');
strNum = num2str(numSteps)
title('Non-Gavity Magnitude of accleration over time with Peak find')
xlabel('time/seconds')
ylabel('m/s^2')
legend('Original Non-Gravity Magnitude of Acceleration', ['Crossing Zero: ' strNum], 'location','northoutside')
hold off
saveas(gcf,'figure6.png')


%% Peak Method Without Filtering

minPeakHeight = std(magNoG);
%minPeakHeight = 1;
[pks, locs] = findpeaks(magNoG, 'MINPEAKHEIGHT', minPeakHeight);

numSteps = numel(pks);

figure
plot(time, magNoG)
hold on
%plot(time(zeroCrossingIndex), zeros, 'b', 'Marker', 'v', 'LineStyle', 'none');
%plot(time(zeroCrossingNIndex), zerosN, 'b', 'Marker', 'v', 'LineStyle', 'none');
plot(time(locs), pks, 'r', 'Marker', 'v', 'LineStyle', 'none');
strNum = num2str(numSteps)
title('Non-Gavity Magnitude of accleration over time with Peak find')
xlabel('time/seconds')
ylabel('m/s^2')
legend('Original Non-Gravity Magnitude of Acceleration', ['Peaks: ' strNum], 'location','northoutside')
hold off
saveas(gcf,'figure7.png')

%% Peak Method With Filtering
figure
plot(time, filteredMagNoG)
minPeakHeight = std(filteredMagNoG);
[pks, locs] = findpeaks(filteredMagNoG, 'MINPEAKHEIGHT', minPeakHeight);
numSteps = numel(pks)
strNum = num2str(numSteps)
hold on
plot(time(locs), pks, 'r', 'Marker', 'v', 'LineStyle', 'none');
title('Non-Gavity Magnitude of accleration over time with Peak find')
xlabel('time/seconds')
ylabel('m/s^2')
legend('Filtered Non-Gravity Magnitude of Acceleration', ['Peaks: ' strNum], 'location','northoutside')
hold off
saveas(gcf,'figure8.png')


%% Speed Analysis
%{
velocity = 0 + cumtrapz(time,magNoG);
positions = 0 + cumtrapz(time, velocity);

figure
plot(time, velocity, time, filteredMagNoG)
figure
plot(time, positions)
%}
