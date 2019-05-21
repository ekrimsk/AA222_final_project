function [vecout] = halfseries(num)

% dont want to grow array so just loop through twice
count = 0; 
numcopy = num;
totsum = 0; 

while totsum < num
	totsum = totsum + round(numcopy/2);
	numcopy = numcopy - round(numcopy/2);
	count = count + 1; 
end 

vecout = zeros(1, count); 
for i = 1:(count)
	vecout(i) = round(num/2);
	num = num - round(num/2);
end 