function [d, D, Q] = getMatrices(forceProfile)
%**************************************************************************
% Function:
%	getMatrices 
%	
%	Description:
%		Provides matrices that are used in evaluating control costs 
%
%	Inputs:
%		forceProfile - structure with force and displacement data 
%		
%	Outputs:
%		d - incremental displacement vector 
%		D - incremental displacement matrix  (upper triangular)
%		Q - matrix for subtracting subsequent control inputs
%
%
%	Revisions:
%		6/21/18 - changed D to match branch and bound better 
%
% 	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 2/07/18
% 		Stanford University, Biomechatronics Lab 
%**************************************************************************
k  = forceProfile.steps; 
force = reshape(forceProfile.force, k, 1); % needs to be a col vector 
disps = reshape(forceProfile.disp, k, 1); % col vec 

d = diff([disps; 0]);  
% D = triu(repmat(d, 1, k));  % before 6/21/18
D = [zeros(k, 1), triu(repmat(d, 1, k - 1))];  % for displacements 
Q = diag(ones(k,1)) -  diag(ones(k-1,1), 1); %	 for change costs 
