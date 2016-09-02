
function d=dtw1(s,t)
% s: signal 1
% t: signal 2
% d: resulting distance

ns=length(s);
nt=length(t);


%% Initialization
D=zeros(ns+1,nt+1)+Inf; % cache matrix
D(1,1)=0;


%% Dynamic programming
for i=1:ns
    for j=1:nt
        d=abs(s(i)-t(j)); % get difference between each point
        D(i+1,j+1)=d+min( [D(i,j+1), D(i+1,j), D(i,j)] ); % input the difference into the matrix
    end
end
d=D(ns+1,nt+1); % the total difference is the last element of the matrix
