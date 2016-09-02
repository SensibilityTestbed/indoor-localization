
% clear all data
clear all;
close all;



% Initial setup

Dsize = 200; % DTW window size, in experiment should be 200 ~ 500
DTW_threshold = 100; % DTW coeefficient > 100 = irrelevant

F = [];
D = [];

mean_D = [];


%% File loading

namestring1 = '_ref.csv'; 
namestringR1 = '_ref.csv';
who = 'yanyan';

resamp_rate = 22; %resampling rate in hz
original_rate = 50; %sony's sampling rate is 50hz, samsung if 100hz

% File load
for s_d = {'c','call', 'hand', 't'}
%for s_d = {'call'}
    string = char(s_d);
    namestring = [who '\ref\' who '_' string];

    for s_r = {'c','call', 'hand', 't'}
    %for s_r = {'call'}
        string = char(s_r);
        namestringR = [who '\ref\' who '_' string];

        if strcmp(s_d, 'call') % for missed experiment files
            numberrange = {'1','2','3','4','5'};
        else
            numberrange = {'1','2','3','4','5'};
        end
        
        for tt = numberrange
            string = char(tt);
            filename = [namestring string namestring1]

            if strcmp(s_r, 'call')      %for missed reference files
                numberrange1 = {'1','2','3','4','5'};
            else
                numberrange1 = {'1','2','3','4','5'};
            end
            for kk = numberrange1
                string = char(kk);
                filenameR = [namestringR string namestringR1] % ref file
                if strcmp(filename,filenameR) %if reference file is same as experiment file, skip
                    continue
                end

                %% Raw data processing
                % Still failed to write into function, can not return an array

                % DTW reference data for trousers
                % Only need accelerometer's data
                dataR = csvread(filenameR,1,0); % all reference file saved as csv
                timeR = dataR(:,1) * 10^-9;
                % Accelerometer data
                axR = dataR(:,2); % For better DTW result, reading data length same as window size
                ayR = dataR(:,3);
                azR = dataR(:,4);

                % Magnititude of acceleration of reference file
                magR = sqrt(axR.^2+ayR.^2+azR.^2);
                % Non-gravity magnititude of accleration
                magNoGR = magR - mean(magR);
                
                Fs_r = length(timeR) / (timeR(length(timeR)) - timeR(1));
                F = [F, Fs_r];
                
                % Experiment data load
                data_temp = csvread(filename,1,0); % all reference file saved as csv
                
                
                
                
                %% Resampling
                data = resample(data_temp, resamp_rate, original_rate); % resampled data
                Fs = resamp_rate; % resamling frequency
                
                
                
               %% Data reading
                
                count = 1:length(data); % setup counter array
                time = data(:,1) * 10^-9; % each sample's collect time
                ax = data(:,2); % accelerometer x-axis
                ay = data(:,3); % accelerometer y-axis
                az = data(:,4); % accelerometer z-axis

                % Magnititude of acceleration data from experiment data
                mag = sqrt(ax.^2+ay.^2+az.^2);
                % Non-gravity magnititude of accleration
                magNoG = mag - mean(mag);

                % Samping frequency of experiment
                



              %% Low pass filter Filter
                % All experiment data and reference data go through the same
                % lowpass fileter

                % Low pass filter
                Flp=3; % low pass filter cutofff frequency 0.8HZ
                [b,a]=butter(5,Flp*2/Fs_r,'low'); % Butter lowpass filter setup

                % Still failed to write these in functions
                % filtfilt function to cutoff the initial bias of filter

                LPFmagR = filtfilt(b,a,magNoGR); % LPF output of non-g magnititude of Trouseres reference acceleration


                [b,a]=butter(5,Flp*2/Fs,'low');
                LPFmag = filtfilt(b,a,magNoG); % LPF output of non-g magnititude of acceleration

                %% Moving average filter
                % Experiment data also go throught the moving average filter
                % Window size settup as 4
                a = 4;
                b = ones(1, a)/a;

                % Moving average filtering

                filteredMagNoG = filter(b, a, LPFmag);
                filteredMagNoGR = filter(b, a, LPFmagR);

                %% Dynamic Time Warping for total file


                D = [D, dtw(filteredMagNoG, filteredMagNoGR)];


            end
        end
        D
        avg_D = mean(D);  % average of the dtw coefficient

        mean_D = [mean_D, avg_D];  % output of the confusion matrix

        D = [];

    end
end

mean_D  % show the confusion matrix in Matlab main page
