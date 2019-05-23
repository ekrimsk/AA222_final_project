%%******************************************************************
%	Energy Recycling Actuator Design Optimization
%	AA222 Final Project Main Script, Spring 2019
%
%	Erez Krimsky 
%   ekrimsky@stanford.edu
%*******************************************************************

close all; clear all; clc; 
Design_Opti_Params;   % TODO clean up 
springLims.w_max = ClutchSpringConstants.cw; 

% ------ Define force profile attributes ------------ 
maxForce = 300;         % Newtons 
maxDisp = 0.04;         % meters 
load('force_profiles/profile_set');


fp = fp_set(1);         % TODO -- replace with more!!!
numProfiles = numel(fp);

for i = 1:numProfiles
    fp{i}.force = maxForce * fp{i}.force/max(fp{i}.force);
    fp{i}.disp = maxDisp * fp{i}.disp/max(fp{i}.disp);
end 


%%
%
%
%   Get Pareto Frontier
%       Will vary weights w and [1 - w] on the two components
%       Good to have things well scaled 
%        
%

num_weights = 10; 
weight_vec = linspace(0, 1, num_weights); 




w = 0.5;    % relative weighting 



% Will want to save some things for post processing 

%%--------  NO Hybrid Opti -----------------


n = 4;      % TODO later, loop through different problem sizes 


% need to find a good initialization point -- do in driver 




% outputs -- fitness, optActuator 



% Add in small plot showing stiffness design space just as a reference here 
% fig = figure; 

%  [minForce, maxForce] = springBoundsPlot(fig, springLims)

% maybe change the "data" param 
[fitness, data, genData] = actuator_optimization(n, fp, w, springLims);





% COMPARING OPTIMIZATIONS USE w = 0.5; 







% will want comparison between plain cma and hybrid apprach(es)



% 