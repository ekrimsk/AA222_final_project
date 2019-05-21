function [minvals, idxs] = mulitimin(array, num)
%**************************************************************************
% 	Function:
%		multimin.m
%
%	Description:
%		Gives 'num' lowest values in an array and their indices
%
%	Inputs:
%		array - input array
%		num - number of mins to find 
%
%	Outputs:
%		minvals - the minimum values in the array, sorted in ascending order
%		idxs - the indices of the min values in the original array
%
%	Notes:
%		If num == 1 this method just calls min, if num is greater than or  
%		equal to the length of the original array, this calls sort
%
%	Revisions:
%		7/5/18 - added speed improvements for edge cases 
%
% 	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 1/12/18 (around)
% 		Stanford University, Biomechatronics Lab 
%**************************************************************************
l = length(array);


if num >= l
	[minvals, idxs] = sort(minvals, 'ascend');
elseif num > 1
	minvals = array(1:num); 
	idxs = (1:num)';

	[maxMin, maxidx] = max(minvals); 
	for i = (num + 1):l
		if (array(i)) < maxMin 
			minvals(maxidx) = array(i); 
			idxs(maxidx) = i; 
		    [maxMin, maxidx] = max(minvals); 
		end 
	end 
	[minvals, I] = sort(minvals, 'ascend');
	idxs = idxs(I);
else
	[minvals, idxs] = min(array);
end 
