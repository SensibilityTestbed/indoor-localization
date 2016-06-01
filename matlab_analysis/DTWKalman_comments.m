%% Indoor localization by using mobile's sensors
% Without using GPS or Wifi, only using mobile internal accelerometer,
% gyroscope and magnetometer. Using peak searching / zero crossing of
% accelerometer's curve to detect steps; Dynamic Time warpping to detect
% device held method and irrelevant motions; External Kalman filter
% estimates the heading

% clear all data
clear all;
close all;

%% File loading
% I tried to write them in a function file, but it only can read last
% element, I didn't solve the problem

% Initial setup
HEIGHT = 1.75; % Personal Height, should be load from file
Dsize = 200; % DTW window size, in experiment should be 200 ~ 500
DTW_threshold = 100; % DTW coeefficient > 100 = irrelevant


% File load, I will keep trying to write them in a function
filename = 'sony_trousers_hand_mv1.csv'; % experiment file
filename1 = 'kalman_new.csv'; % Kalman output file
filenameRT = 't2.csv'; % DTW trousers reference file
filenameRH = 'sony_hand.csv'; % DTW hand reference file


%% Raw data processing
% Still failed to write into function, can not return an array

% DTW reference data for trousers
% Only need accelerometer's data
dataRT = csvread(filenameRT,1,0); % all reference file saved as csv

% Accelerometer data
axRT = dataRT(1:Dsize,2); % For better DTW result, reading data length same as window size
ayRT = dataRT(1:Dsize,3);
azRT = dataRT(1:Dsize,4);

% Magnititude of acceleration of reference file
magRT = sqrt(axRT.^2+ayRT.^2+azRT.^2);

% Non-gravity magnititude of accleration
magNoGRT = magRT - mean(magRT);


% DTW reference data for hand
dataRH = csvread(filenameRH,1,0);
axRH = dataRH(1:Dsize,2);
ayRH = dataRH(1:Dsize,3);
azRH = dataRH(1:Dsize,4);

% Magnititude of acceleration
magRH = sqrt(axRH.^2+ayRH.^2+azRH.^2);
magNoGRH = magRH - mean(magRH);

% Kalman output data
kalman = csvread(filename1,1,0);
k_theta = kalman(:,1);
% k_theta_bias = mean(kalman(1:100));
% cut off the initial bias

% Experiment data load
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

% Magnititude of gyroscope data from experiment data
gyro = sqrt(gx.^2+gy.^2+gz.^2);

% Samping frequency of experiment
Fs = length(time) / (time(length(time)) - time(1));


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
LPFMx = filtfilt(b, a, mx);
LPFMy = filtfilt(b, a, my);
LPFMz = filtfilt(b, a, mz);
LPFGx = filtfilt(b, a, gx);
LPFGy = filtfilt(b, a, gy);
LPFGz = filtfilt(b, a, gz);
LPFmag = filtfilt(b,a,magNoG); % LPF output of non-g magnititude of acceleration
LPFmagRT = filtfilt(b,a,magNoGRT); % LPF output of non-g magnititude of Trouseres reference acceleration
LPFmagRH = filtfilt(b,a,magNoGRH); % LPF output of non-g magnititude of hand reference acceleration
filteredk_theta = filtfilt(b,a,k_theta); % LPF output of kalman filter output 
% filteredk_theta not used in this script, LPF should be setup before
% Kalman output

LPFGyro = filtfilt(b,a,gyro); % LPF output raw gyroscope data

%% Moving average filter
% Experiment data also go throught the moving average filter
% Window size settup as 4
a = 4;
b = ones(1, a)/a;

% Moving average filtering
filteredAx = filtfilt(b, a, LPFAx);  %Moving average filter output for accelerometer x-aix
filteredAy = filtfilt(b, a, LPFAy);
filteredAz = filtfilt(b, a, LPFAz);
filteredMx = filtfilt(b, a, LPFMx);
filteredMy = filtfilt(b, a, LPFMy);
filteredMz = filtfilt(b, a, LPFMz);
filteredGx = filter(b, a, LPFGx);
filteredGy = filter(b, a, LPFGy);
filteredGz = filter(b, a, LPFGz);
filteredMagNoG = filter(b, a, LPFmag);
filteredMagNoGRT = filter(b,a,LPFmagRT);
filteredMagNoGRH = filter(b,a,LPFmagRH);

%% Dynamic Time Warping
% Repeated compare the experiment data with reference data with samle
% length, and save in two list

DistListT = [];
DistListH = [];

% get DTW coefficients
% for example, the length of experiment data is 2600, and Dsize = 500, 
% then the for loop will run 5 times, cover 1-2500 samples 
for i = 1: (floor(length(magNoG)/Dsize)) % 
    DT=dtw(filteredMagNoG(((i-1)*Dsize+1) : i*Dsize), filteredMagNoGRT);
    DH=dtw(filteredMagNoG(((i-1)*Dsize+1) : i*Dsize), filteredMagNoGRH);
    DistListT = [DistListT, DT];
    DistListH = [DistListH, DH];
end

% get rest DTW coefficients
% for above example, below codes get rest 100 samples DTW coefficients
DT=dtw(filteredMagNoG((floor(length(magNoG)/Dsize))*Dsize+1 : length(magNoG)), filteredMagNoGRT);
DH=dtw(filteredMagNoG((floor(length(magNoG)/Dsize))*Dsize+1 : length(magNoG)), filteredMagNoGRH);
DistListT = [DistListT, DT];
DistListH = [DistListH, DH];


%% Carrying method detection by Gravity detection
% old script for carrying method detection
% Instead by DTW carring method detection
%{
% hand
if abs(mean(az)) > sqrt(mean(ax).^2 + mean(ay).^2) * 2
    gyroFlag = 0;
    thetaFlag = 1;
else
    % trousers
    if abs(mean(ay)) > sqrt(mean(ax).^2 + mean(az).^2) * 2
        gyroFlag = 2;
        thetaFlag = -1;
    % coat
    else
        gyroFlag = 1;
        thetaFlag = 1;
    end
end
%}


%% RAW Gyro data processing

% From DTW coefficient, choose different Gyro axis data
head = [];
for i = 1:length(LPFGx)
    itr = floor(i/Dsize)+1;
    if DistListT(itr) < DistListH(itr) && DistListT(itr) < DTW_threshold % If DTW detect carrying method is Trousers
        head = [head, LPFGy(i)]; % Using gyro y axis data
    elseif DistListH(itr) < DistListT(itr) && DistListH(itr) < DTW_threshold % If DTW detect carrying method is hand
        head = [head, LPFGz(i)]; % Using gyro z axis data
    else
        head = [head, 0]; % ignore irrelevant data
        % ignore irrelevant data is incorrect, but this raw gyro data is
        % just a reference curve to compete with Kalman output
    end
end

% heading from Gyro is equal to integer of raw Gyro data
% setup head in 2pi
head = mod((0 + cumtrapz(time,head)), 2*pi);


%% Raw magnetomer data processing
% another referece data
% much more inaccurate
% project heading into global x-y plain

direction = [];
headDx = [];
headDy = [];

for i = 1:length(time)
    itr = floor(i/Dsize)+1;
    if DistListT(itr) < DistListH(itr) && DistListT(itr) < DTW_threshold % Put in trousers
        headDx = [headDx, LPFMx(i)]; % using magnetometer x axis as global magne x axis
        headDy = [headDy, LPFMy(i)]; % using magnetometer y axis as global magne y axis
    elseif DistListH(itr) < DistListT(itr) && DistListH(itr) < DTW_threshold % held in hand
        headDx = [headDx, LPFMx(i)]; % using magnetometer x axis as global x axis
        headDy = [headDy, LPFMz(i)]; % using magnetometer z axis as globals y axis
    else % irrelevant motions
        headDx = [headDx, headDx(length(headDx))]; % same as put int trousers
        headDy = [headDy, headDy(length(headDy))]; 
        % incorrect to setup as trousers, but who cares...
    end
end


% project magenetomer to heading
for i = 1:length(time)
    tempM = headDx(i)/headDy(i); 
    if headDy(i) > 0
        direction = [direction, pi/2 - atan(tempM)];
    elseif headDy(i) < 0
        direction = [direction, 1.5*pi - atan(tempM)];
    else
        if headDx(i) < 0
            direction =[direction, pi];
        else
            direction =[direction, 0.0];
        end
    end
end


%% Kalman estimate
% Matlab Kalman filter estimation
% Different with Seth's algorithm
% not too accurate
% will add comments later

P = [1 0; 0 1];
R_angle = 0.3;

% Q_angle = 0.001;
% Q_gyro = 0.003;
Q_angle = 0.05;
Q_gyro = 0.5;
Q = [Q_angle 0; 0 Q_gyro];

A = [0 -1; 0 0];
q_bias = 0; % Initialize gyro bias
angle = 0; % Initialize gyro angle
q_m = 0;
X = [0; 0];
x1 = zeros(size(time));
x2 = zeros(size(time));
for i=1:length(time),
    q_m = direction(i);
    q = q_m - q_bias; % /* Pitch gyro measurement */
    Pdot = A*P + P*A' + Q;
    rate = q;

    angle = angle + q/Fs;
    P = P + Pdot/Fs;
    C = [1 0];
    angle_err = head(i)-angle;
    
    E = C*P*C' + R_angle;
    K = P*C'*inv(E);
    P = P - K*C*P;

    X = X + K * angle_err;
    x1(i) = X(1);
    x2(i) = X(2);
    angle = x1(i);
    q_bias = x2(i);
end

heading = x1;


%% heading from Kalman estimation and raw gryoscope data
% heading = interger of raw gyro scope

ThetaGx = mod((0 + cumtrapz(time,LPFGx)), 2*pi); % get x-axis heading
ThetaGy = mod((0 + cumtrapz(time,LPFGy)), 2*pi); % get y-axis heading
ThetaGz = mod((0 + cumtrapz(time,LPFGz)), 2*pi); % get z-axis heading
ThetaMag = mod(sqrt((0 + cumtrapz(time,LPFGx)).^2 + (0 + cumtrapz(time,LPFGy)).^2 + (0 + cumtrapz(time,LPFGz)).^2), 2*pi);
% get heading from gyroscope's magnititude


gyroX = mod(ThetaGx, 2*pi) * 180/pi;
gyroY = mod(ThetaGy, 2*pi) * 180/pi;
gyroZ = mod(ThetaGz, 2*pi) * 180/pi;

thetaGyro = heading;

DTWMagNoG = [];
for i = 1:length(filteredMagNoG)
    if DistListT(floor(i/Dsize)+1)<DTW_threshold || DistListH(floor(i/Dsize)+1)<DTW_threshold
        DTWMagNoG = [DTWMagNoG, filteredMagNoG(i)];
    else
        DTWMagNoG = [DTWMagNoG, 0];
    end
end

DTWMagNoG = DTWMagNoG.'

%{
%% PCA heading


pcaS = []
windowSize = 6028;
for i = 1:floor(length(LPFGx)/windowSize)
    floor(length(LPFGx)/windowSize)
    
    itr = (i-1)*windowSize;
    x=LPFAx(itr+1:itr+windowSize);
    y=LPFAy(itr+1:itr+windowSize);
    z=LPFAz(itr+1:itr+windowSize);

    x1=LPFGx(itr+1:itr+windowSize);
    y1=LPFGy(itr+1:itr+windowSize);
    z1=LPFGz(itr+1:itr+windowSize);
    
    data1 = [x1 y1 z1];

    [COEFFS, SCORES] = princomp(data1);
    pcaS = [pcaS; SCORES(:,2)];
end
data1 = [gx(i*windowSize+1 : length(LPFGx)) gy(i*windowSize+1 : length(LPFGx)) gz(i*windowSize+1 : length(LPFGx))];
[COEFFS, SCORES] = princomp(data1);
pcaSS = [pcaS; SCORES(:,2)];
PCA_angle = mod((0 + cumtrapz(time,pcaSS)), 2*pi)+pi;
%}
%% Zero Crossing Method

n = length(DTWMagNoG);
ind = 1:(n-1);

%minPeakHeight = std(filteredMagNoG);
minPeakHeight = 0.1;

y = DTWMagNoG > minPeakHeight;

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

numStepsZ = length(xc);

xctemp = [xc(1)];

for i = 2:numStepsZ
    if xc(i) - xc(i-1) >= 0.25
        xctemp = [xctemp, xc(i)];
    end
end

xc = xctemp;
numStepsZ = length(xc);



%% Peak Method Without Filtering

%minPeakHeight = std(magNoG);
minPeakHeight = 0.1;

[pkstemp, locstemp] = findpeaks(DTWMagNoG, 'MINPEAKHEIGHT', minPeakHeight);

numStepsP = numel(pkstemp);
locs = [locstemp(1)];
pks = [pkstemp(1)]

for i = 2:numStepsP
    if time(locstemp(i)) - time(locstemp(i-1)) >= 0.25
        locs = [locs, locstemp(i)];
        pks = [pks, pkstemp(i)];
    end
end
numStepsP = length(locs);


%% Indoor localization by Gyro Intergration

stride = HEIGHT * 0.43
fZ = numStepsZ / (time(k(numStepsZ))-time(k(1)));
fP = numStepsP / (time(locs(numStepsP))-time(locs(1)));

%strideZ = HEIGHT * 0.43 + 0.16 * fZ - 0.32;
%strideP = HEIGHT * 0.43 + 0.14 * fP - 0.28;

strideZ = HEIGHT * 0.43;
strideP = HEIGHT * 0.43;

positionX = [0];
positionY = [0];

for i = 2:numStepsZ
    lastx = positionX(length(positionX));
    lasty = positionY(length(positionY));
    theta = thetaGyro(k(i));
    dPositionX = strideZ*cos(theta);
    dPositionY = strideZ*sin(theta);
    positionX = [positionX lastx+dPositionX];
    positionY = [positionY lasty+dPositionY];
    
end


positionX1 = [-1.21];
positionY1 = [0.8636];

for i = 2:numStepsP
    lastx = positionX1(length(positionX1));
    lasty = positionY1(length(positionY1));
    theta = thetaGyro(locs(i));    
    dPositionX = strideP*cos(theta);
    dPositionY = strideP*sin(theta);
    positionX1 = [positionX1 lastx+dPositionX];
    positionY1 = [positionY1 lasty+dPositionY];
    
end


%% Indoor localization by Kalman output


KalmanPosX = [0];
KalmanPosY = [0];

headingk = [];

for i = 1:length(k_theta)
    itr = floor(i/Dsize)+1;
    if DistListT(itr) < DistListH(itr) && DistListT(itr) < DTW_threshold
        headingk = [headingk, k_theta(i)+pi];
    elseif DistListH(itr) < DistListT(itr) && DistListH(itr) < DTW_threshold
        headingk = [headingk, k_theta(i)];
    else
        headingk = [headingk, 0];
    end
end


costheta = cos(headingk);
sintheta = sin(headingk);
for i = 2:numStepsP
    lastx = KalmanPosX(length(KalmanPosX));
    lasty = KalmanPosY(length(KalmanPosY));
    %theta = (k_theta(locs(i))- k_theta(1)) * thetaFlag ;
    %theta = (k_theta(locs(i))- k_theta(1)) * thetaFlag * -1

    %dKalmanPosX = stride*mean(costheta(locs(i)-25 : locs(i)+25));
    %dKalmanPosY = stride*mean(sintheta(locs(i)-25 : locs(i)+25));
    
    dKalmanPosX = strideP*costheta(locs(i));
    dKalmanPosY = strideP*sintheta(locs(i));
    
    KalmanPosX = [KalmanPosX lastx+dKalmanPosX];
    KalmanPosY = [KalmanPosY lasty+dKalmanPosY];
    
end
%{
%% PCA localization

positionXpca = [-1.21];
positionYpca = [0.8636];

for i = 2:numStepsP
    lastx = positionXpca(length(positionXpca));
    lasty = positionYpca(length(positionYpca));
    theta = PCA_angle(locs(i));    
    dPositionX = strideP*cos(theta);
    dPositionY = strideP*sin(theta);
    positionXpca = [positionXpca lastx+dPositionX];
    positionYpca = [positionYpca lasty+dPositionY];
    
end
%}

%% kalman estimation
%{
perfectStep = round(20.8/stride)+1;

posPx = [0];
posPy = [0];

for i = 2:perfectStep
    lastx = posPx(length(posPx));
    posPx = [posPx lastx+stride];
    posPy = [posPy 0];
end

perfectStep = round(9.24/stride)+2;
for i = 2:perfectStep
    lastx = posPx(length(posPx));
    lasty = posPy(length(posPy));
    theta = 1.34156
    dPositionX = stride*cos(theta);
    dPositionY = stride*sin(theta);
    posPx = [posPx lastx+dPositionX];
    posPy = [posPy lasty+dPositionY];
end

perfectStep = round(22.9/stride)+2;
for i = 2:perfectStep
    lastx = posPx(length(posPx));
    lasty = posPy(length(posPy));
    posPx = [posPx lastx-stride];
    posPy = [posPy lasty];
end

perfectStep = round(9/stride)+2;
for i = 2:perfectStep
    lastx = posPx(length(posPx));
    lasty = posPy(length(posPy));
    posPx = [posPx lastx];
    posPy = [posPy lasty - stride];
end

if numStepsP > length(posPx)
    posKx = (positionX1(1:length(posPx)) + posPx)/2;
    posKy = (positionY1(1:length(posPx)) + posPy)/2;
else
    posKx = (positionX1 + posPx(1:numStepsP))/2;
    posKy = (positionY1 + posPy(1:numStepsP))/2;
end
  %}  

%% Figure
figure
subplot(2,1,1)
plot(count, ax)
hold on
plot(count,ay, 'r')
plot(count,az, 'g')
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
title('Magnetic field Magnitude over time')
xlabel('time/seconds')
ylabel('micro tesla')
legend('X-axis', 'Y-axis', 'Z-axis' );
hold off


subplot(2,1,2)
plot(time, filteredMx)
hold on
plot(time,filteredMy, 'r')
plot(time,filteredMz, 'g')
title('Filtered Magnetic field Magnitude over time')
xlabel('time/seconds')
ylabel('micro tesla')
legend('X-axis', 'Y-axis', 'Z-axis' );
hold off

saveas(gcf,'figure3.png')


figure

subplot(2,1,1)
plot(time, ThetaGx)
hold on
plot(time,ThetaGy, 'r')
plot(time,ThetaGz, 'g')
title('Intergraled filtered Gyro Magnitude over time in radians')
xlabel('time/seconds')
ylabel('radians')
legend('X-axis', 'Y-axis', 'Z-axis' );
hold off

subplot(2,1,2)
plot(time, gyroX)
hold on
plot(time,gyroY, 'r')
plot(time,gyroZ, 'g')
title('Intergraled filtered Gyro Magnitude over time in angle')
xlabel('time/seconds')
ylabel('angle')
legend('X-axis', 'Y-axis', 'Z-axis' );
hold off

saveas(gcf,'figure4.png')




figure
subplot(2,1,1)
plot(time, filteredMagNoG)
hold on
plot(xc, zeros(length(xc)), 'r', 'Marker', 'x', 'LineStyle', 'none');
strNum = num2str(numStepsZ);
title('Non-Gavity Magnitude of accleration over time with Zero Crossing detection')
xlabel('time/seconds')
ylabel('m/s^2')
legend('Non-Gravity Magnitude of Acceleration', ['step counts by Crossing Zero: ' strNum], 'location','northoutside')
hold off

subplot(2,1,2)
plot(time, filteredMagNoG)
hold on
%plot(time(zeroCrossingIndex), zeros, 'b', 'Marker', 'v', 'LineStyle', 'none');
%plot(time(zeroCrossingNIndex), zerosN, 'b', 'Marker', 'v', 'LineStyle', 'none');
plot(time(locs), pks, 'r', 'Marker', 'v', 'LineStyle', 'none');
strNum = num2str(numStepsP);
title('Non-Gavity Magnitude of accleration over time with Peak find detection')
xlabel('time/seconds')
ylabel('m/s^2')
legend('Non-Gravity Magnitude of Acceleration', ['Step counts by Peak Searching: ' strNum], 'location','northoutside')
hold off
saveas(gcf,'figure5.png')

figure
plot(positionX, positionY, 'b-v', positionX1, positionY1, 'r-o', KalmanPosX, KalmanPosY, 'm-*')

title('Indoor path tracking')
xlabel('meter')
ylabel('meter')
legend('BY crossing zero algorithm path tracking', 'BY peak searching algorithm path tracking', 'Kalman output', 'location', 'southoutside')
saveas(gcf,'figure6.png')


figure
subplot(3,1,1)
plot(time, headingk)
hold on
title('kalman_heading')
xlabel('time')
ylabel('radians')
legend('headng')
hold off
subplot(3,1,2)
plot(time, head)
title('heading angle')
xlabel('time')
ylabel('radians')
legend('kalman_output')
subplot(3,1,3)
plot(time, direction)
title('magnometer direction')
xlabel('time')
ylabel('radians')
legend('headng')
saveas(gcf,'figure7.png')


figure
hold on

for i = 1:length(KalmanPosX)
    plot(KalmanPosX(i), KalmanPosY(i), 'b-o')
    plot(KalmanPosX(1:i),KalmanPosY(1:i), 'b-o')
    pause
end

title('Indoor path tracking')
xlabel('meter')
ylabel('meter')
%legend('lab boundary', 'ideal path','in trousers pocket','take out of the pocket','held in hand' ,'location', 'southoutside')
hold off
saveas(gcf,'figure8.png')

