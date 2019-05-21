function rowvec = row(invec)
%*************************************************************************
%	Function:
%		row.m
%
%	Description:
%		Takes a vector or matrix and transposes it if it has more cols
%		than rows 
%
%	Inputs:
%		invvec - input vector/matrx
%
%	Outpus:
%		rowvec - output
%
%	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 4/10/18 
% 		Stanford University, Biomechatronics Lab 
%*************************************************************************
if size(invec, 1) > size(invec, 2)
	rowvec = invec';
else
	rowvec = invec;
end
