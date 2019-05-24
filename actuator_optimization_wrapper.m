%%******************************************************************
%	Energy Recycling Actuator Design Optimization
%	AA222 Final Project Main Script, Spring 2019
%
%	Erez Krimsky 
%   ekrimsky@stanford.edu


% 		n - 
%		w - 
% 		type - 'basic', 'lamarckian', or 'baldwinian'
%*******************************************************************


% may want to rethink outputs and 

function [fitness, optimal_design, data, genData] = actuator_optimization_wrapper(n, w, opt_type)

% Wrapper for optimization, passes most of the parameters in 
% Sets the scale for the forces and the displacements 
% 



Design_Opti_Params;   % TODO clean up 
springLims.w_max = ClutchSpringConstants.cw; 

% ------ Define force profile attributes ------------ 
maxForce = 300;         % Newtons 
maxDisp = 0.04;         % meters 
load('force_profiles/profile_set');


fp = fp_set(1);         % TODO -- replace with more!!! ---- 
numProfiles = numel(fp);

for i = 1:numProfiles
    fp{i}.force = maxForce * fp{i}.force/max(fp{i}.force);
    fp{i}.disp = maxDisp * fp{i}.disp/max(fp{i}.disp);
end 


if strcmp(opt_type, 'none')
	[fitness, optimal_design, data, genData] = actuator_optimization(n, fp, w, springLims);
else
	[fitness, optimal_design, data, genData] = actuator_optimization(n, fp, w, springLims, opt_type);
end 