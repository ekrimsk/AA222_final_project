function numout = binvec2num(binvec)
%**************************************************************************
%	Function:
%		binvec2num
% 	
%	Description:
%		Converts a binary vector (like [0, 1, 0, 1]) to an integer. Used 
%		in conjunction with num2binvec.
%	
%	Inputs:
%		binvec - vector cintaining 1s and zeros
%
%	Outputs:
%		numout - the integer that the binary representation corresponds to
%
%	Example:	
%		numout = binvec2num([0, 1, 0, 1]);
%		numout now contains 5.
%
% 	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 1/24/18
% 		Stanford University, Biomechatronics Lab 
%**************************************************************************
numout = 0; 
bits = numel(binvec);
for i = bits:-1:1
	if binvec(i)
		numout = numout + 2^(bits - i);
	end
end
