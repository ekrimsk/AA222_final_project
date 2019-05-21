function [minStiff, maxStiff] = stiffBounds(springLims, f0)
%******************************************************************************
%	Function:
%		stiffBounds.m
%
%	Description:
%		Determines minimum and maximum feasible stiffeness for for set
%		of nominal inital forces (f0)
%
%	Inputs:
%		springLims - structure with geometry limits for springs 
%		f0 - vector of initial forces 
%	
%	Outputs:
%		minStiff - vector of min stiffs corresponding to each force 
%		maxStiff - vector of max stiffs corresponding to each force 
%
%	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 3/11/18 
% 		Stanford University, Biomechatronics Lab 
%******************************************************************************
np = length(f0); 


t_min = springLims.t_min;
t_max = springLims.t_max;

w_min = springLims.w_min;
w_max = springLims.w_max;

ps_min = springLims.ps_min;
ps_max = springLims.ps_max; 

L_min = springLims.L_min;
L_max = springLims.L_max;

A_min = t_min * w_min;
A_max = t_max * w_max;

k1 = ClutchSpringConstants.k1; 
k2 = ClutchSpringConstants.k2; 


%% Min stiffness criteria 
for i = 1:np
	f_val = f0(i);
	L_val = L_max; 
	ps_val = ps_max; 
	A_val = (f_val/(L_val * ps_val)) / ( (k1/L_val) + (k2/(L_val * ps_val))); 
	if (A_val > A_max)
		A_val = A_max; 
	elseif (A_val < A_min)
		A_val = A_min; 
		ps_val = (f_val - (k2 * A_val)) / (k1 * A_val);
	end

	minStiff(i) = k1 * A_val/L_val;
end 


%% Max stiffness criteria 
for i = 1:np
	f_val = f0(i);
	L_val = L_min; 
	ps_val = ps_min; 

	A_val = (f_val/(L_val * ps_val)) / ( (k1/L_val) + (k2/(L_val * ps_val))); 

	if (A_val > A_max)
		A_val = A_max; 
		ps_val = (f_val - (k2 * A_val)) / (k1 * A_val);
	elseif (A_val < A_min)
		A_val = A_min; 
		ps_val = (f_val - (k2 * A_val)) / (k1 * A_val);
	end

	maxStiff(i) = k1 * A_val/L_val;
end 
