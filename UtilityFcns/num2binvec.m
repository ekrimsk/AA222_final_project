function binvec = num2binvec(num, numbits)
%*****************************************************************************
%	Function:
%		num2binvec
%
%	Description:
%		Converts a number to a binary representation stored in a vector.
%		Used in conjunction with binvec2num
%	
%	Inputs:
%		num - a number (integer) to convert to a binary representation
%		numbits - number of bits (entries in output vector) for conversion
%
%	Outputs:
%		binvec - vector (1 x numbits) containing 0s and 1s representing 
%				the input number. The LSB is at the end of the vector and 
%				the MSB is at the beginning.
%
%	Example:	
%		vec = num2binvec(7, 4);
%		vec now contains [0, 1, 1, 1]
%
% 	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 1/24/18
% 		Stanford University, Biomechatronics Lab 
%*****************************************************************************


binvec = zeros(1, numbits);

for i = 1:numbits
	den = 2^(numbits - i);
	bit = floor(num/den);
	num = num - bit*den; 
	binvec(i) = bit; 
end


