function [y_dot] = midpointDiff(y, dt)
%**************************************************************************
% 	Function:
%		midpointDiff.m
%
%	Description:
%		
%
%	Inputs:
%		
%		
%	Outputs:
%		
%
%	Notes:
%		
%	Revisions:
%
% 	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 7/03/18
% 		Stanford University, Biomechatronics Lab 
%**************************************************************************
y_dot = zeros(size(y, 1), size(y, 2));  
if size(y, 2) > size(y, 1)  % MORE COLS 
	diffs = diff(y, 1, 2);  
	y_dot(:, 2:end - 1) = 0.5 * (diffs(:, 1:end - 1) + diffs(:, 2:end))/dt;
	y_dot(:, 1) = diffs(:, 1)/dt; 
	y_dot(:, end) = diffs(:, end)/dt; 
else    					% MORE ROWS 				
	diffs = diff(y, 1, 1);  
	y_dot(2:end - 1, :) = 0.5 * (diffs(1:end - 1, :) + diffs(2:end, :))/dt;
	y_dot(1, :) = diffs(:, 1)/dt; 
	y_dot(end, :) = diffs(end, :)/dt; 
end
