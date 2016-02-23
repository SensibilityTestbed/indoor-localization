clear all
close all

%% File loading and data processing
filename = 'c_2_4_20_20.csv';
M = csvread(filename,1,0)
time = M(:,1)
X = M(:,2)
Y = M(:,3)
Z = M(:,4)
mag = sqrt(X.^2+Y.^2+Z.^2);
%magNoG = mag - 9.81;
magNoG = mag - mean(mag) %10.7233 for 4th

%% Figure
figure
plot(time,magNoG)
title('Non-Gavity Magnitude acceleration over time')
xlabel('time/seconds')
ylabel('m/s^2')

%% Moving Average Filter
samplingPerStep = 10;
coeff = ones(1, samplingPerStep)/samplingPerStep;
filteredMagNoG = filter(coeff, 1, magNoG);  %Moving average filter

hold on
plot(time, filteredMagNoG, 'r')
legend('Without filter','Filtered')
hold off
saveas(gcf,'figure1.png')

%% Threshold Crossing Method

n = length(filteredMagNoG);
ind = 1:(n-1);

minPeakHeight = std(filteredMagNoG)/2;
%minPeakHeight = 1.5

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
saveas(gcf,'figure2.png')


%% Crossing Threshold Method Without Filtering


n = length(magNoG);
ind = 1:(n-1);

minPeakHeight = std(magNoG);
%minPeakHeight = 1.5

y = magNoG > minPeakHeight;

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
plot(time, magNoG)
hold on
plot(xc, zeros(length(xc)), 'r', 'Marker', 'v', 'LineStyle', 'none');
strNum = num2str(numSteps)
title('Non-Gavity Magnitude of accleration over time with Peak find')
xlabel('time/seconds')
ylabel('m/s^2')
legend('Original Non-Gravity Magnitude of Acceleration', ['Crossing Zero: ' strNum], 'location','northoutside')
hold off
saveas(gcf,'figure3.png')


sd = std(filteredMagNoG)
m = mean(filteredMagNoG)
sd_raw = std(magNoG)
m1 = mean(magNoG)


