function [totCost, forceCost, changeCost, posCost] = computeCost(forceError,...
						     kf, changeMat, changeCosts, kc, posWeight, kp, dt)
%*************************************************************************
%	Function:
%		computeCost.m
%
%   Description:
%		Computes cost function based off...
%		forceError - equal to (requestedForce - suppliedForce)
% 		totChange - number of clutches that changed states 
%		numEngaged - number of engaged clutches 
%		posWeight - weight function (defined in Actuator.m) that assigns high
%					weight to clutches that are close to bottoming out
%
%	Inputs:
%		forceError - vector of force error 
%		kf - force error weighting factor 
%		changeMat - binary matrix of clutch changes (0 = no change)
%	    changeCosts - clutch change cost vector 
%		kc -  clutch change cost weighting factor
%		posWeight - raw position cost, computed by calling function
%		kp - position cost weighting factor 
%		dt - time difference between steps 
%
%	Outputs:
%		totCost - total cost
%		forceCost - force component of total cost
%		changeCost - clutch change component of total costs
%		posCost - position component of total cost
%
%	Revisions:
%		3/15/18 - added 'dt' term to fix scaling problem 		
%
%	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 1/16/18 
% 		Stanford University, Biomechatronics Lab 
%*************************************************************************


%% Position Weight - see definition in calling function
if isinf(posWeight)
	posCost = inf;
else
	posCost = kp * dt * posWeight; 
end

if ~isrow(forceError) && ~iscolumn(forceError)
	error('Force error passed in as matrix'); 
end 

%% Force Error - Using quadratic right now 
forceCost =  dt * kf * sum(forceError.^2); 


%% Total Change
tmp1 = (changeMat.^2);
tmp2 = changeCosts * tmp1;
changeCost = kc * sum(tmp2);  



totCost = forceCost + changeCost + posCost; 
