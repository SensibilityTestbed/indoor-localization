function f = fact(x)

temp = 0;

if (x == 1)
    temp = 1;
else
    temp = x*fact(x-1);
end


f = temp;