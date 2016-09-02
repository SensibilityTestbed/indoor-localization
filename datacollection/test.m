clear all;
close all;

timelist = [1,2,3,4,5,6,7,8,9,10];
index = [];
for time = 1:0.1:9
    index = [index, binary(timelist, time)];
end

i=1;
for time = 1:0.1:9
    temp_string = fprintf('time: %f, index: %f\n', time, index(i));
    disp(temp_string);
    i = i+1;
end
