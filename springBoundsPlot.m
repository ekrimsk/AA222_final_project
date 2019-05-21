function [minForce, maxForce] = springBoundsPlot(figHandle, springLims)
%**************************************************************************
%	Function:
%		springBoundsPlot
%
%	Description:
%		Creates plot with stiffness on y axis and force on x-axis showing
%		a shaded green region of valid force-stiffness pairs 		
%
%	Inputs:
%		figHandle - handle to figure where plot will be made 
%		springLims - structure with spring geometry consrtaints 
%
%	Outputs:
%		minForce - minimum f0 for these spring limits 
%		maxForce - maximum f0 for these spring limits 
%
%   Revisions:
%		3/21/18 - robustness + readability 
%		
%
%	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 4/2/18
% 		Stanford University, Biomechatronics Lab 
%**************************************************************************
set(0, 'CurrentFigure', figHandle), hold on 
k1 = ClutchSpringConstants.k1; 
k2 = ClutchSpringConstants.k2; 

%%  ----- Copy Parameters from the structure (for code readability) ------
ps_min = springLims.ps_min;
ps_max = springLims.ps_max;
L_min = springLims.L_min;
L_max = springLims.L_max;
w_min = springLims.w_min;
w_max = springLims.w_max;
t_min = springLims.t_min;
t_max = springLims.t_max;
A_min = w_min * t_min;
A_max = w_max * t_max;	


% Compute max and min forces 
minForce = (k2 * A_min) + (k1 * ps_min * A_min); 
maxForce= (k2 * A_max) + (k1 * ps_max * A_max); % N/m 


kl_min = k1 * A_min/L_max; 
A_val = (maxForce/(L_max * ps_max)) / ( (k1/L_max) + (k2/(L_max * ps_max))); 
kl_max = k1 * A_val/L_max; 

f_lower_vert = ((ps_max * k1) + k2) * A_min; 
f_lower = [minForce, f_lower_vert, maxForce];
k_lower = [kl_min, kl_min, kl_max];


ku_max = k1 * A_max/L_min; 
A_val = (minForce/(L_min * ps_min)) / ( (k1/L_min) + (k2/(L_min * ps_min))); 
ku_min = k1 * A_val/L_min; 
f_upper_vert = ((ps_min * k1) + k2) * A_max; 

f_upper = [minForce, f_upper_vert, maxForce];
k_upper = [ ku_min, ku_max, ku_max];


fillx = [f_lower, fliplr(f_upper)];
filly = [k_lower, fliplr(k_upper)]; 

h = fill(fillx, filly,'g', 'DisplayName', 'Bounds'); 
set(h,'facealpha',.5);

KU = plot(f_lower, k_lower, 'k', 'linewidth', 2, 'HandleVisibility', 'off');
KL = plot(f_upper, k_upper, 'k', 'linewidth', 2, 'HandleVisibility', 'off');


xlabel('Force (N)');
ylabel('Stiffness (N/m)'); 
xlim([0 maxForce]);
ylim([0 ku_max*1.05]);
