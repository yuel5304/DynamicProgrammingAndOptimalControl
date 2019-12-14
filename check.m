function flag = check(P,u)
%CHECK Summary of this function goes here
%   Detailed explanation goes here

tmp = size(P);
sum = 0;
for i = 1:tmp(1)
    sum = sum + P(1,i,u);
end
flag = sum;
return
end

