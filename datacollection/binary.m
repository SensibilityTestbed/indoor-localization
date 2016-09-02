function i = binary(timelist, time)
len = length(timelist);
index = round(len/2);

if (len <= 2)
    if timelist(1) > time
        i = 0;
    else
        i = 1;
    end
elseif (timelist(index) == time)
    i = index;
else
    if time > timelist(index)
        i = index - 1 + binary(timelist(index:length(timelist)), time);
    else
        i = binary(timelist(1:index), time);
    end
end
