function totCost = costFromMatrices(C, D, f, b, S, kf, kc, kp, dt, lims);
%*************************************************************************
% 	Function:
%		costFromMatrices.m		
%
%	Description:
%		computes performance cost just from matrix inputs 
%
%	Inputs:
%		C - control matrix
%		D - matrix defining displacements 
%		f - target forces
%		b - nominal spring forces
%		S - nominal spring stifnesses
%		kf - force error weight factor
%		kc - change cost weight factor 
%		kp - postion cost weight factor
% 		dt - time step 
%		lims - displacement limits (2 x 1) where lims(1) < lims(3)
%
%	Outputs:
%		totCost - total cost
%
% 	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 4/2/18
% 		Stanford University, Biomechatronics Lab 
%**************************************************************************
changeCosts = b' + S*max(lims); 

n = length(b);
P = C*D; 
outForce = C'*b + (S*(C.*P))'; % variable force 
forceError = f(:) - outForce(:); 

changeMat = diff([zeros(n, 1), C], 1, 2); 	%  assume clutches were initially off 
posWeight = computeRawPosCost(P, lims, n); 

totCost =  computeCost(forceError,...
					  kf, changeMat, changeCosts, kc, posWeight, kp, dt);

if max(size(totCost)) ~= 1
	errorStruct.message = 'Returned non scalar cost';
	errorStruct.identifier = 'costFromMatrices:incorrectSizeInputs';
	error(errorStruct);
end 
