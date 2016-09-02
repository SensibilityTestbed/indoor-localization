
% clear all data
clear all;
close all;



% Initial setup

Dsize = 200; % DTW window size, in experiment should be 200 ~ 500
DTW_threshold = 100; % DTW coeefficient > 100 = irrelevant


D = [];

mean_D = [];


%% File loading

namestring1 = '_ref.csv'; 
namestringR1 = '_ref.csv';

who = 'jessica';

% File load
%for s_d = {'b', 'hand', 't'}
for s_d = {'call'}
    string = char(s_d);
    namestring = [who '\ref\' who '_' string];

    for s_r = {'b', 'hand', 't'}
    %for s_r = {'call'}
        string = char(s_r);
        namestringR = [who '\ref\' who '_' string];


        for tt = {'2','5'}
            string = char(tt);
            filename = [namestring string namestring1]

            for kk = {'1','2','3','4','5'}
                string = char(kk);
                filenameR = [namestringR string namestringR1] % ref file


                %% Raw data processing
                % Still failed to write into function, can not return an array

                % DTW reference data for trousers
                % Only need accelerometer's data
                dataR = csvread(filenameR,1,0); % all reference file saved as csv

                % Accelerometer data
                gxR = dataR(:,5); % For better DTW result, reading data length same as window size
                gyR = dataR(:,6);
                gzR = dataR(:,7);

                % Magnititude of acceleration of reference file
                magR = sqrt(gxR.^2+gyR.^2+gzR.^2);

                % Experiment data load
                data = csvread(filename,1,0);
                count = 1:length(data); % setup counter array
                time = data(:,1) * 10^-9; % each sample's collect time
                gx = data(:,5); % accelerometer x-axis
                gy = data(:,6); % accelerometer y-axis
                gz = data(:,7); % accelerometer z-axis

                % Magnititude of acceleration data from experiment data
                mag = sqrt(gx.^2+gy.^2+gz.^2);

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

                LPFmag = filtfilt(b,a,mag); % LPF output of non-g magnititude of acceleration
                LPFmagR = filtfilt(b,a,magR); % LPF output of non-g magnititude of Trouseres reference acceleration





                %% Dynamic Time Warping for total file


                D = [D, dtw(LPFmag, LPFmagR)];


            end
        end
        D
        avg_D = mean(D);

        mean_D = [mean_D, avg_D];

        D = [];

    end
end

mean_D


