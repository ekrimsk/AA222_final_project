function stringout = starprint(stringin)
%*************************************************************************
%	Function:
%		starprint.m
%
%	Description:
%		Prints out the words in the middle of a line of asterisks 
%
%	Outputs:
%		stringout - the string that this function just printed to the 
%		screen		
%
%	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 3/12/18 
% 		Stanford University, Biomechatronics Lab 
%*************************************************************************

if(isempty(stringin))
	stringout = repmat('*', 1, 79); 
else
	numchar = length(stringin); 
	ll = 79; % 80 chars 
	pad = 2; 
	numFirst = floor((ll - numchar)/2) - pad; 
	numLast = ll - numFirst - 2 * pad - numchar; 

	padstr = repmat(' ', 1, pad); 
	firstStr = repmat('*', 1, numFirst);
	lastStr = repmat('*', 1, numLast);
	stringout = sprintf('%s%s%s%s%s', firstStr, padstr, stringin, padstr, lastStr);
end
