%% Multi files plots

namestring = 'yu\quarternion_ref\yu_';
namestring1 = '_ref_q.csv';



std_x = [];
std_y = [];
std_z = [];

mean_x = [];
mean_y = [];
mean_z = [];

for st = {'call1','hand1','t1','b1'}
    string = char(st);
    filename = [namestring string namestring1];
   

   
    data = csvread(filename,1,0);

    % Read quaternion data
    x = data(:,1); 
    y = data(:,2);
    z = data(:,3);
    s = data(:,4);
    
    for i = 1: length(x)
        sum_temp = x(i).^2 + y(i).^2 + z(i).^2;
        if sum_temp ~= 1.0
            x(i) = (1.0/sum_temp).^0.5 * x(i);
            y(i) = (1.0/sum_temp).^0.5 * y(i);
            z(i) = (1.0/sum_temp).^0.5 * z(i);
        end
    end
    
    std_x = [std_x, std(x)];
    std_y = [std_y, std(y)]
    std_z = [std_z, std(z)]
    
    mean_x = [mean_x, mean(x)];
    mean_y = [mean_y, mean(y)];
    mean_z = [mean_z, mean(z)];
    
end

save('var_mean.txt','std_x','std_y','std_z','mean_x','mean_y','mean_z','-ascii')