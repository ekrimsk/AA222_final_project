%*************************************************************************
%	Function:
%		vecprint.m
%
%	Description:
%		Creates an easily printable string from a vector		
%
%	Inputs:
%
%
%	Outputs:
%
%
%	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 3/12/18 
% 		Stanford University, Biomechatronics Lab 
%*************************************************************************
function stringout = vecprint(vector, numdec)

if (size(vector, 1) ~= 1) && (size(vector, 2) ~= 1)
	error('The input vector was a matrix or empty')
end 


np = length(vector);
stringin = repmat(['%0.', num2str(numdec),'f,  '], 1, np); 
stringin = stringin(1:end - 3); 
stringout = sprintf(stringin, vector);
