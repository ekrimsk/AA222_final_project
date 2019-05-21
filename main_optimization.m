%%***************************
%
%
%***************************

close all; clear all; clc; 


% ------ Define force profile attributes ------------ 
load('force_profiles/profile_set');
maxForce = 300;         % Newtons 
maxDisp = 0.04;         % meters 

fp = fp_set(1);         % TODO -- replace with more!!!
numProfiles = numel(fp);

for i = 1:numProfiles
    fp{i}.force = maxForce * fp{i}.force/max(fp{i}.force);
    fp{i}.disp = maxDisp * fp{i}.disp/max(fp{i}.disp);
end 


% LOAD IN SPRING LIMS 
Design_Opti_Params;   % TODO -- clean this up -- remove what we dont really want to use here or split 

springLims.w_max = ClutchSpringConstants.cw; 
%%
%
%
%   Get Pareto Frontier
%       Will vary weights w and [1 - w] on the two components
%       Good to have things well scaled 
%        
%

w = 0.5;    % relative weighting 



%%--------  NO Hybrid Opti -----------------


n = 5;      % TODO later, loop through different problem sizes 


% need to find a good initialization point -- do in driver 




% outputs -- fitness, optActuator 



% Add in small plot showing stiffness design space just as a reference here 
fig = figure; 

%  [minForce, maxForce] = springBoundsPlot(fig, springLims)

% maybe change the "data" param 
[fitness, data] = actuator_optimization(n, fp, w, springLims);

% will want comparison between plain cma and hybrid apprach(es)



% 