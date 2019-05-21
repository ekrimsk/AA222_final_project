
close all; clear all; clc; 

%{
 One thing we want to show is what do trajectories look like with an 
intial force and stiffness with an optimized controller 
%}    


% Load in profiles -- NOTE, it could be interesting to replace these with orthog poly script 





% will need something that defines the controller function we are calling -- use anonymous function handles so we can easily change 



% define control 


% really need bqf to be centered though 










% INTIALIZATION POINT -- forces on a 2^n strcuture (if allowable)
% stiffness at midrange 
% when this leads to forces to small we will fix 
% actually -- probably more reliable to linsapce from min to (near) max as initialization





Design_Opti_Params; 

n = 4; 

%% ------ Load in a series of force-displacement profiles to test with -----
load('force_profiles/profile_set');
fp = fp_set(1); 
numProfiles = numel(fp);


lims = [-10, 10]    % TODO obviously not good 


%% -------------- Define Scale of force for opti -----------------

k1 = ClutchSpringConstants.k1;
k2 = ClutchSpringConstants.k2;

% Compute max and min force based off geometry 
minForce_geom = (k2 * A_min) + (k1 * ps_min * A_min); 
maxForce_geom = (k2 * A_max) + (k1 * ps_max * A_max); % N/m 


forceProfile = fp{1}; 


f_vec = linspace(minForce_geom, max(forceProfile.force)/1.5, n)';

[minStiff, maxStiff] = stiffBounds(springLims, f_vec)

k_vec = 0.5 * (minStiff + maxStiff);  
k_vec = k_vec(:); 

% So many friggin inputs on the actuator object 

test_actuator = Actuator(n, 'vector', f_vec, 'S', k_vec,...
                                'limits', lims, 'kp', 1e-5, 'kc', 1e-5); 
                                
%{
optActuator = Actuator(n, 'vector', f_vec, 'S', k_vec,...
                                'limits', optiData.lims,...
                                'kf', kf, 'kc', kc, 'kp', kp,...
                                'w', w_list,...
                                't', t_list,...
                                'L0', L_list,...
                                'L_cs', optiData.L_cs,...
                                'L_co', optiData.L_co,...
                                'L_cf', optiData.L_cf,...
                                'ps', optiData.ps,...
                                'L_spring_offset', optiData.L_spring_offset,...
                                'output_init', optiData.output_init,...
                                'output_max', optiData.output_max,...
                                'output_min', optiData.output_min);
%} 



%%  for now just want to be able to pass in fp to control optimizer 


domRatio = inf; % 10000; % f this shit honestly 
numControl = 70;        % shouldn=t really need this -- abstract hese things out 
numKeep = 16; 

[bestSols, forces, minvals, varargout]  =  BQBF(test_actuator,...
                                        forceProfile, numKeep, numControl, domRatio) 

U = bestSols{1}; 

% TODO NEED TO ADD A WRAPPER AROUND BQBF TO ABSTRACT THIS NONSE OUT AND JUST RETURN U AND F 

f_control = forces{1}; 
t = forceProfile.time; 
f_des = forceProfile.force; 


% Apply the gradiented version 

% X here is the postions -- can calcluate this out from the control as the P matrix 
% need accees to D tooooo   -- maybe then move this into the fixed control min function then?



% need to create D matrix from displacements -- based on how many control points we used 

% MVOE THIS SOMEWHERE SMART 

% With writing it out like this, last U shouldnt matter 

Up = U(:, 1:end-1); % U priome 
d = diff(forceProfile.disp)';   % col vec
[~, h] = size(Up); 

D = triu(repmat(d, [1, h])); % be careful with row vec vs col v 

% NEED TO INCLUDE THE x0 term here -- because really x0 

x0 = zeros(n, 1);

X = [x0, Up*D];        % need to account for an initial position too ! 


f_des_mids = (f_des(1:end-1) + f_des(2:end))/2; 








f_check =  (f_vec'*U) + k_vec'*(U .* X);      % should be same as force returned by bqbf 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%








% Ignoring the fact that our optimizer is acting on the worong thing 
% we have 70 control values 
% We have 70 force points specfied




% we will evaluate at 69 -- ignore ther last control 

% lets compute the displacements and see if what the optimizer did stilll makes sense for 
% for what we have 

X_mid = 0.5 * (X(:, 1:end-1) + X(:, 2:end)); 



f_opt_mid = (f_vec'*Up) + k_vec'*(Up .* X_mid);  


t_mid = 0.5*(t(1:end-1) + t(2:end)); 


% f_des_mids -- 


% calculate objective value error for this initial design 


obj_init = norm(f_opt_mid - f_des_mids, 2); 

display(obj_init);




% now lets call our "optimizer" and see wtf is up 




[f_vec_new, k_vec_new, f_me] = fixed_control_min(f_des_mids, Up, X_mid, f_vec, k_vec); 

f_opt_mid_new  = (f_vec_new'*Up) + k_vec_new'*(Up .* X_mid);  


figure, 
%plot(t, f_des, 'r'), hold on 
plot(3 * t_mid, f_des_mids, 'k', 'DisplayName', 'Desired Force'), hold on 
plot(3 * t_mid, f_opt_mid, 'b', 'DisplayName', 'Initial Force Output');
plot(3 * t_mid, f_opt_mid_new, 'r', 'DisplayName', 'Force Output After Optimization')

ylabel('Force (N)')
xlabel('Time (s)')
legend show 
print('milestone_fig', '-dpng', '-r300'); 
hold off  



%%%%%%%%%%%%%%%%%%














%{




[f_vec_new, k_vec_new, f_me] = fixed_control_min(f_des_mids, Up, X, f_vec, k_vec); 


f_new = (f_vec_new'*U) + k_vec_new'*(U .* X);

% TODO after dinner -- control function 
%       outputs -- f, U, X ---- will decide if midpoint forces or not, probs dont 



% 


% TODO -- add in the step where we apply convex optimization to improve things 



figure, plot(t, f_des, 'r'), hold on 
plot(t, f_control, 'b')
plot(t, f_new, 'k')
plot(t, f_check, 'g')
plot(0.5*(t(1:end-1) + t(2:end)), f_me, 'm')
hold off  

%}