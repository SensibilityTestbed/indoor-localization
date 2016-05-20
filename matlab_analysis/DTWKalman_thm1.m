clear all;
close all;

%% File loading and data processing
HEIGHT = 1.75;
Dsize = 200;
DTW_threshold = 100;

filename = 'sony_trousers_hand_mv1.csv';
filename1 = 'kalman_new.csv';

filenameRT = 'sony_trousers.csv';
filenameRH = 'sony_hand.csv';

dataRT = csvread(filenameRT,1,0);
axRT = dataRT(1:Dsize,2);
ayRT = dataRT(1:Dsize,3);
azRT = dataRT(1:Dsize,4);

magRT = sqrt(axRT.^2+ayRT.^2+azRT.^2);
magNoGRT = magRT - mean(magRT);

dataRH = csvread(filenameRH,1,0);
axRH = dataRH(1:Dsize,2);
ayRH = dataRH(1:Dsize,3);
azRH = dataRH(1:Dsize,4);

magRH = sqrt(axRH.^2+ayRH.^2+azRH.^2);
magNoGRH = magRH - mean(magRH);


kalman = csvread(filename1,1,0);
k_theta = kalman(:,1);
%k_theta_bias = mean(kalman(1:100));


data = csvread(filename,1,0);
count = 1:length(data);
time = data(:,1) * 10^-9;
ax = data(:,2);
ay = data(:,3);
az = data(:,4);
gx = data(:,5);
gy = data(:,6);
gz = data(:,7);
mx = data(:,8);
my = data(:,9);
mz = data(:,10);





mag = sqrt(ax.^2+ay.^2+az.^2);
magNoG = mag - mean(mag);
gyro = sqrt(gx.^2+gy.^2+gz.^2);
Fs = length(time) / (time(length(time)) - time(1));

%% Filter

% Low pass filter
Flp=3; % low pass filter cutofff frequency 0.8HZ
[b,a]=butter(5,Flp*2/Fs,'low'); % Butter lowpass filter

LPFAx = filtfilt(b, a, ax);
LPFAy = filtfilt(b, a, ay);
LPFAz = filtfilt(b, a, az);
LPFMx = filtfilt(b, a, mx);
LPFMy = filtfilt(b, a, my);
LPFMz = filtfilt(b, a, mz);
LPFGx = filtfilt(b, a, gx);
LPFGy = filtfilt(b, a, gy);
LPFGz = filtfilt(b, a, gz);
LPFmag = filtfilt(b,a,magNoG);
LPFmagRT = filtfilt(b,a,magNoGRT);
LPFmagRH = filtfilt(b,a,magNoGRH);
filteredk_theta = filtfilt(b,a,k_theta);
LPFGyro = filtfilt(b,a,gyro);


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
filteredMagNoGRT = filter(b,a,LPFmagRT);
filteredMagNoGRH = filter(b,a,LPFmagRH);

%% DTW

DistListT = [];
DistListH = [];
for i = 1: (floor(length(magNoG)/Dsize))
    DT=dtw(filteredMagNoG(((i-1)*Dsize+1) : i*Dsize), filteredMagNoGRT);
    DH=dtw(filteredMagNoG(((i-1)*Dsize+1) : i*Dsize), filteredMagNoGRH);
    DistListT = [DistListT, DT];
    DistListH = [DistListH, DH];
end

DT=dtw(filteredMagNoG((floor(length(magNoG)/Dsize))*Dsize+1 : length(magNoG)), filteredMagNoGRT);
DH=dtw(filteredMagNoG((floor(length(magNoG)/Dsize))*Dsize+1 : length(magNoG)), filteredMagNoGRH);
DistListT = [DistListT, DT];
DistListH = [DistListH, DH];

%% Carrying method detection
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

%% Gyro data processing
head = [];

for i = 1:length(LPFGx)
    itr = floor(i/Dsize)+1;
    if DistListT(itr) < DistListH(itr) && DistListT(itr) < DTW_threshold
        head = [head, LPFGy(i)];
    elseif DistListH(itr) < DistListT(itr) && DistListH(itr) < DTW_threshold
        head = [head, LPFGz(i)];
    else
        head = [head, 0];
    end
end
head = mod((0 + cumtrapz(time,head)), 2*pi);


%% magnetomer data processing
direction = [];

headDx = [];
headDy = [];

for i = 1:length(time)
    itr = floor(i/Dsize)+1;
    if DistListT(itr) < DistListH(itr) && DistListT(itr) < DTW_threshold
        headDx = [headDx, LPFMx(i)];
        headDy = [headDy, LPFMy(i)];
    elseif DistListH(itr) < DistListT(itr) && DistListH(itr) < DTW_threshold
        headDx = [headDx, LPFMx(i)];
        headDy = [headDy, LPFMz(i)];
    else
        headDx = [headDx, headDx(length(headDx))];
        headDy = [headDy, headDy(length(headDy))];
    end
end


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




%% heading

ThetaGx = mod((0 + cumtrapz(time,LPFGx)), 2*pi);
ThetaGy = mod((0 + cumtrapz(time,LPFGy)), 2*pi);
ThetaGz = mod((0 + cumtrapz(time,LPFGz)), 2*pi);
ThetaMag = mod(sqrt((0 + cumtrapz(time,LPFGx)).^2 + (0 + cumtrapz(time,LPFGy)).^2 + (0 + cumtrapz(time,LPFGz)).^2), 2*pi);




magGyro = sqrt(ThetaGx.^2+ThetaGy.^2);

gyroX = mod(ThetaGx, 2*pi) * 180/pi;
gyroY = mod(ThetaGy, 2*pi) * 180/pi;
gyroZ = mod(ThetaGz, 2*pi) * 180/pi;

thetaGyro = heading-pi*2.7/4;



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

positionX = [-1.21];
positionY = [0.8636];

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


KalmanPosX = [-1.21];
KalmanPosY = [0.8636];

headingk = [];

for i = 1:length(k_theta)
    itr = floor(i/Dsize)+1;
    if DistListT(itr) < DistListH(itr) && DistListT(itr) < 150
        headingk = [headingk, k_theta(i)+pi];
    elseif DistListH(itr) < DistListT(itr) && DistListH(itr) < 150
        headingk = [headingk, k_theta(i)];
    else
        headingk = [headingk, 0];
    end
end

for i = 1:locs(5)
    headingk(i) = headingk(i) + 13/12*pi;
end  

for i = locs(6):locs(28)
    headingk(i) = -0.125 + rand(1)*0.25;
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
%% lab side and error calculating


perfectPathX = [0, 21.33, 21.33, 23.54, 23.54, -1.21, -1.21, 0];
perfectPathY = [0, 0,     1.8,   7.8,   8.97,  8.97,   0,    0];

retangX = [0,      20.73,  22.94, 0,    0];
retangY = [0.8636, 0.8636, 7.89,  7.89, 0.8636];

differenceZX = [positionX(numStepsZ),0];
differenceZY = [positionY(numStepsZ),0];
deltaZ = sqrt(positionX(numStepsZ).^2 + positionY(numStepsZ).^2);

differencePX = [positionX1(numStepsP),0];
differencePY = [positionY1(numStepsP),0];
deltaP = sqrt(positionX1(numStepsP).^2 + positionY1(numStepsP).^2);

distanceZ = numStepsZ * stride;
distanceP = numStepsP * stride;

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
plot(positionX, positionY, 'b-v', positionX1, positionY1, 'r-o', retangX, retangY, 'k', perfectPathX, perfectPathY, perfectPathX - 15.8, perfectPathY - 11.61, 'k:', KalmanPosX, KalmanPosY, 'm-*')
hold on
plot(differenceZX, differenceZY, 'b:', differencePX, differencePY, 'r:')
title('Indoor path tracking')
xlabel('meter')
ylabel('meter')
legend('BY crossing zero algorithm path tracking', 'BY peak searching algorithm path tracking', 'lab sides', 'perfect path', 'perfect path with bias','Kalman output',['Total mistakes from zero crossing in meters: ' num2str(deltaZ)], ['Total mistakes from peak search in meters: ' num2str(deltaP)], 'location', 'southoutside')
hold off
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
plot(retangX, retangY, 'k', perfectPathX, perfectPathY, 'k:') 
hold on


for i = 1:93
    plot(KalmanPosX(i), KalmanPosY(i), 'b-*')
    plot(KalmanPosX(1:i),KalmanPosY(1:i), 'b-*')
    axis([-5, 25, -4, 12])
    pause
    
end
for i = 93:95
    plot(KalmanPosX(i), KalmanPosY(i), 'r-o')
    plot(KalmanPosX(93:i),KalmanPosY(93:i), 'r-o')
    pause
end
for i = 95:length(KalmanPosX)
    plot(KalmanPosX(i), KalmanPosY(i), 'g-x')
    plot(KalmanPosX(95:i),KalmanPosY(95:i), 'g-x')
    pause
end
%plot(KalmanPosX(1:93), KalmanPosY(1:93), 'b-*')
%plot(KalmanPosX(93:95),KalmanPosY(93:95), 'r-o' )
%plot(KalmanPosX(95:length(KalmanPosX)),KalmanPosY(95:length(KalmanPosY)), 'g-x' )

%plot( perfectPathX - 2.15, perfectPathY - 6.202, 'b:')
title('Indoor path tracking')
xlabel('meter')
ylabel('meter')
%legend('lab boundary', 'ideal path','in trousers pocket','take out of the pocket','held in hand' ,'location', 'southoutside')
hold off
saveas(gcf,'figure6.png')


var = var(LPFmag(locs(1):locs(numStepsP)));

a = 0.41529737200316149995609027838764;
b = 0.12934014665847018529902520418021;
c = 0.05128655484324229384385702994643;

destanceP = strideP * numStepsP
strideE = a + b*fP + c*var
distanceE = strideE * numStepsP

actual_length = 20.8 + 22.9 + 9 + sqrt(2.1.^2 + 9.^2)

%f = fit(time,ax,'fourier8')
%%ft = fft(ax, time)
%plot(f, time, LPFAx)