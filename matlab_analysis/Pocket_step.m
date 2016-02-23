clear all
close all

%% File loading and data processing
filename = '1_26_14_34_matlab.csv';
M = csvread(filename,1,0)
time = M(:,1)
X = M(:,2)
Y = M(:,3)
Z = M(:,4)
mag = sqrt(X.^2+Y.^2+Z.^2);
magNoG = mag - 9.8;
%magNoG = mag - mean(mag) %10.7233

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

%% Zero Crossing Method
%abovezero = magNoG > 0;
%zeroCrossing = diff(abovezero) == 1;
%zeroCrossingNegative = diff(abovezero) == -1;
%[zeroCrossingIndex,zeros] = find(zeroCrossing);
%[zeroCrossingNIndex,zerosN] = find(zeroCrossingNegative);
%numberOfSteps = numel(zeroCrossingIndex) + numel(zeroCrossingNIndex)

%% Low Pass Filter
A = [1]
B = [1 -0.95]; % 0.95 to 0.97 pre-emphasis filter
filteredMagNoG = filter(B,A,magNoG);
%{
Fs=25000; % sampling frequency
Fhp=10000; % low pass filter cutofff frequency 10 Hz
[b,a]=butter(5,Fhp/Fs,'low'); % Butter lowpass filter
filteredMagNoG = filter(b,a,magNoG); 
%}

hold on
plot(time, filteredMagNoG, 'r')
legend('Without filter','Filtered')
hold off
saveas(gcf,'figure5.png')


%% Moving average filter
%{
a = 1;
b = zeros(1, 50);
for t=1:50
    b(1,t)=1/50
end

filteredMagNoG = filter(a,b,magNoG);

%}
%% Kalman Filter
%{
DeltaT = 0.1;
F = [1 DeltaT; 0 1];
G = [DeltaT^2/2; DeltaT];
H = [1 0];
sigma_a = 0.1;
Q = sigma_a^2;
R = 0.1;

p0 = 100*eye(2,2);
x0 = [0;0];

xx(:,1) = x0;
pp = p0
pp_norm(1) = norm(pp);

for t=1:length(magNoG)
    [x1,p1,x,p] = kalm(magNoG(t),xx(:,t),pp,F,G,H,Q,R);
    xx(:,t+1) = x1;
    pp = p1;
    pp_norm(t+1) = norm(pp);
end
figure
plot(time, pp)
%}

%% Peak Method Without Filtering
%minPeakHeight = std(magNoG);
minPeakHeight = 1;
[pks, locs] = findpeaks(magNoG, 'MINPEAKHEIGHT', minPeakHeight);

invMagNoG = magNoG * -1;
[pks1, locs1] = findpeaks(invMagNoG, 'MINPEAKHEIGHT', minPeakHeight);

numSteps = numel(pks)+numel(pks1);
figure
plot(time, magNoG)
hold on
%plot(time(zeroCrossingIndex), zeros, 'b', 'Marker', 'v', 'LineStyle', 'none');
%plot(time(zeroCrossingNIndex), zerosN, 'b', 'Marker', 'v', 'LineStyle', 'none');
plot(time(locs), pks, 'r', 'Marker', 'v', 'LineStyle', 'none');
plot(time(locs1), -pks1, 'r', 'Marker', 'v', 'LineStyle', 'none');
strNum = num2str(numSteps)
title('Non-Gavity Magnitude of accleration over time with Peak find')
xlabel('time/seconds')
ylabel('m/s^2')
legend('Original Non-Gravity Magnitude of Acceleration', ['Peak and Valley: ' strNum], 'location','northoutside')
hold off
saveas(gcf,'figure6.png')

%% Peak Method With Filtering
figure
plot(time, filteredMagNoG)
minPeakHeight = std(filteredMagNoG);
invFilteredMagNoG = filteredMagNoG * -1;
[pks, locs] = findpeaks(filteredMagNoG, 'MINPEAKHEIGHT', minPeakHeight);
[pks1, locs1] = findpeaks(invFilteredMagNoG, 'MINPEAKHEIGHT', minPeakHeight);
numSteps = numel(pks)+numel(pks1)
strNum = num2str(numSteps)
hold on
plot(time(locs), pks, 'r', 'Marker', 'v', 'LineStyle', 'none');
plot(time(locs1), -pks1, 'r', 'Marker', 'v', 'LineStyle', 'none');
title('Non-Gavity Magnitude of accleration over time with Peak find')
xlabel('time/seconds')
ylabel('m/s^2')
legend('Filtered Non-Gravity Magnitude of Acceleration', ['Peak and Valley: ' strNum], 'location','northoutside')
hold off
saveas(gcf,'figure7.png')

%% Speed Analysis
%{
velocity = 0 + cumtrapz(time,magNoG);
positions = 0 + cumtrapz(time, velocity);

figure
plot(time, velocity, time, filteredMagNoG)
figure
plot(time, positions)
%}
