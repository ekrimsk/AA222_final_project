%*************************************************************************
%	File:
%		Design_Opti_Params.m
%
%	Description:
%		Adds parameters to workspace for design optimization
%
%	Notes:
%		As of 5/3/18 c1 + c2 + c3 do NOT = 1 
%
%	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 4/16/18 
% 		Stanford University, Biomechatronics Lab 
%*************************************************************************

%% ------------------------- Define Actuator Parameters ---------------------
kc = 0.2; % 0.01; %1.2; 
kf = 1; 
ke = 1; 
kp = 300; %5; 

%%  ------------------- Set bounds on physical dimensions -----------------

%t_min = 0.5e-3; 		% thickness 
%t_max = 2e-3; 			% thickness 
t_min = 0.79e-3;         % thickness 
t_max = 0.79e-3;           % thickness



%w_min = 1e-2; 			% width
%w_max = 35e-2; 			% width 

w_min = 1e-2;           % width
w_max = 11e-2;          % width -- TODO -- link up with cw 

A_min = t_min * w_min; 	% area
A_max = t_max * w_max; 	% area

L_min = 2e-2; 			% unstretched length 
L_max = 50e-2; 			% unstretched length 

%ps_min = 2; 			% prestretch
%ps_max = 2.75; 			% prestretch

% For stuart Check 
ps_min = 2.25; 			% prestretch
ps_max = 3.75; 			% prestretch


%max_stretch = 3.5; 		% TOTAL stretch (NOT prestretch)
%min_stretch = 1.25;		% we dont want to let it go completely slacj  

% For stuart Check 
max_stretch = 4.5; 		% TOTAL stretch (NOT prestretch)
min_stretch = 1.8;		% we dont want to let it go completely slacj  


% Put all of these into a structure that can be passed around 
springLims.t_min = t_min;
springLims.t_max= t_max; 
springLims.w_min = w_min;
springLims.w_max = w_max;
springLims.L_min = L_min;
springLims.L_max = L_max; 
springLims.ps_min = ps_min;
springLims.ps_max = ps_max; 

springLims.max_stretch = max_stretch;
springLims.min_stretch = min_stretch;
