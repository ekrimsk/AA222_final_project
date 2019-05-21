
%*********************************

% fp -- structure of force profiles 
% n = number of springs 
% w = weight on control (1 - w is weight on design)
% spingLims -- bounds on what dimensions springs can reasonably have 

% TODO -- add in options to use the hybrid methods 
% TODO -- add i/o options for tracking results 


%*********************************

% could add optActuator as an output but probs wont do anything with it in the short term 

function [fitness, data] = actuator_optimization(n, fp, w, springLims)

% NOTE flags/options for radial design? 

% NOTE -- an input parser could be really nice here 
% TODO -- assertion check on 0 <= w <= 1


    k1 = ClutchSpringConstants.k1;
    k2 = ClutchSpringConstants.k2;
    cw = ClutchSpringConstants.cw; 
    cf = ClutchSpringConstants.cf; 

t = springLims.t_max; 


% cw = 0.1;      % clutch width --- TODO -- move this somewhere smarter, may need to pass around


% Get the maximum force and displacements from the force profile 
maxForce = -inf; maxDisp = -inf;  minDisp = inf; 

for i = 1:numel(fp)
    if (max(fp{i}.force) > maxForce)
        maxForce =  max(fp{i}.force);
    end 

    if (max(fp{i}.disp) > maxDisp)
        maxDisp = max(fp{i}.disp);
    end 

    if (min(fp{i}.disp) < minDisp)
        minDisp =  min(fp{i}.disp);
    end 
end 

display(maxForce)
display(maxDisp)
display(minDisp)


% Define initial force guess, stifness guess, spring length, clutch length guess 

% TODO -- incorporate back in to have min force guess 
%minForce_geom = (k2 * A_min) + (k1 * ps_min * A_min); 
%maxForce_geom = (k2 * A_max) + (k1 * ps_max * A_max); % N/m 

f_min = 3; % N, lowest we 


f_init = linspace(0.2 * maxForce, 0.5 * maxForce, n)';
[minStiff, maxStiff] = stiffBounds(springLims, f_init);
k_init = minStiff + 0.2 * (maxStiff - minStiff);  % midway stiffness
k_init = k_init(:);  

%w_init = (k_init .* L_init)./(k1 .* t); 
w_init = 0.9 * cw * ones(n, 1);     % just for feasible init 
% L_init = 3 * maxDisp.*ones(n, 1); % TODO issue is that there are limits on what this can be 
L_init = (w_init./k_init)*k1.*t; 


F_max_init = f_init + k_init*maxDisp;

cf = ClutchSpringConstants.cf; 
l_cs_init = 2 * (f_init + maxDisp*k_init)/(cf * cw);



% NOTE -- may want to make a function for combining and separating out the parts of this 
init_design = [f_init; k_init; L_init; l_cs_init];  
display(f_init)
display(k_init)
display(L_init)
display(l_cs_init)

display(init_design)
% ----------------------------
%
% Define function handles to pass to the solver  -- will use penalties 
%
% ----------------------------------

% where 'x' is the design vector 
fitnessfun = @(x) w * design_fitness(x, springLims, maxDisp, minDisp) + (1 - w) * control_fitness(x, fp); 


% Run the CMA -- get the results out an return 

% TODO 
cmaOptions.MaxGenerations = 1000; 
cmaOptions.MaxStallGenerations = 100;   % 80 
parmdiffs = 0.05 * init_design;     % or keep at identity? idk 




% TODO -- add in oother cma options and maybe an arg parser for doing lamarck/baldwinian 


% just looking at where the issues arrise on the initial point 


%design_fitness(init_design, springLims, maxDisp, minDisp); 
%return 



% NOTE: would like to be able to initialize at a feasible point 






% 

doPrint = true;


display(w_init); 
init_penalty = design_penalty(init_design, springLims, maxDisp, minDisp); 

display(init_penalty); 

[final_design, fullCost] = cma(init_design, fitnessfun, cmaOptions, parmdiffs, " ", doPrint);


% some prints 

des_final = design_fitness(final_design, springLims, maxDisp, minDisp) ;

control_final = control_fitness(final_design, fp); 

display(des_final)
display(control_final) 


[f_vec, k_vec, L_vec, l_cs_vec] = split_design(final_design); 

    k1 = ClutchSpringConstants.k1;
    k2 = ClutchSpringConstants.k2;
    cw = ClutchSpringConstants.cw; 
    cf = ClutchSpringConstants.cf; 


% quick compute what else we got here -- TODO == move this out to some other place
l0_vec = (f_vec./k_vec) - (k2/k1)*L_vec; 
tops = l0_vec + maxDisp + l_cs_vec;
F_max_vec = f_vec + (maxDisp .* k_vec); 
l_ol = F_max_vec./(cw * cf);  
% NOTE 2 lines below becomes dfferent for rotary actuator 
l_co = max(max(tops) - (l0_vec + minDisp + l_cs_vec - l_ol)) * ones(n, 1); 
bottoms = min(tops - l_cs_vec, 0); 
d_score = max(max(tops) - min(bottoms));    % effective length 


w_vec = (k_vec .* L_vec)./(k1 .* t); 


% Required overlap 


display(f_vec)
display(k_vec)
display(L_vec)
display(l_cs_vec)
display(l_co)
display(d_score)
display(w_vec);
display(l_ol);
display(F_max_vec);  



final_penalty = design_penalty(final_design, springLims, maxDisp, minDisp); 
display(final_penalty)

%% TODO -- probably move these out to their own places 

end


function [f_vec, k_vec, L_vec, l_cs_vec] = split_design(x)
    x = x(:);  
    n = round(length(x)/4);
    f_vec = x(1:n); 
    k_vec = x(n+1:2*n); 
    L_vec = x(2*n+1:3*n); 
    l_cs_vec = x(3*n + 1:4*n); 

end 


function design_score = design_fitness(x, springLims, maxDisp, minDisp)

    % MIN DISP IS NEGATIVE (or zero)
    k1 = ClutchSpringConstants.k1;
    k2 = ClutchSpringConstants.k2;
    cw = ClutchSpringConstants.cw; 
    cf = ClutchSpringConstants.cf; 
    t = springLims.t_max; 

    [f_vec, k_vec, L_vec, l_cs_vec] = split_design(x); % function to distribute 
    n = length(f_vec); 
    % will need to incorporate some penalties here



    %
    % Compute design score 
    % 

    % NOTE max disp is a scalar here 
    l0_vec = (f_vec./k_vec) - (k2/k1)*L_vec; 
    tops = l0_vec + maxDisp + l_cs_vec;
    F_max_vec = f_vec + (maxDisp .* k_vec); 
    l_ol_req = F_max_vec./(cw * cf);    % this is a REQUIRED l_ol 

    % NOTE 2 lines below becomes dfferent for rotary actuator 
    l_co_vec = max(max(tops) - (l0_vec + minDisp + l_cs_vec - l_ol_req)) * ones(n, 1); 
    bottoms = min(tops - l_co_vec, 0); 
    d_score = max(max(tops) - min(bottoms));    % effective length 


    % then take quadratic and counts function on the penalty 
    rho = 1000; % could add penalty increasing in CMA     
    penalty = design_penalty(x, springLims, maxDisp, minDisp); 
    p_quad = norm(penalty, 2)^2;  
    p_count = nnz(penalty); 
    penalty_cost = p_count + rho*p_quad;
    design_score = d_score + penalty_cost;
end 


function penalty =  design_penalty(x, springLims, maxDisp, minDisp)
  % MIN DISP IS NEGATIVE (or zero)
    k1 = ClutchSpringConstants.k1;
    k2 = ClutchSpringConstants.k2;
    cw = ClutchSpringConstants.cw; 
    cf = ClutchSpringConstants.cf; 
    t = springLims.t_max; 

    [f_vec, k_vec, L_vec, l_cs_vec] = split_design(x); % function to distribute 
    n = length(f_vec); 

    % NOTE max disp is a scalar here 
    l0_vec = (f_vec./k_vec) - (k2/k1)*L_vec; 
    tops = l0_vec + maxDisp + l_cs_vec;
    F_max_vec = f_vec + (maxDisp .* k_vec); 
    l_ol_req = F_max_vec./(cw * cf);    % this is a REQUIRED l_ol 

    % NOTE 2 lines below becomes dfferent for rotary actuator 
    l_co_vec = max(max(tops) - (l0_vec + minDisp + l_cs_vec - l_ol_req)) * ones(n, 1); 
    bottoms = min(tops - l_co_vec, 0); 




    % forces out of order penalty -- need forces increasing 
    force_order_penalty = min(diff(f_vec),0);
    % negative froce penalty 
    neg_f_penalty = min(0, f_vec); 
    % Width only goes negative if length or stifness goes negative 
    neg_L_penalty = min(0, L_vec); 
    % spring clutches cant have negative length 
    neg_l_cs_penalty = min(0, l_cs_vec); 
    % spring too wide penalty -- cant be wider than clutch width 
    w_vec = (k_vec .* L_vec)./(k1 .* t); 
    w_large_penalty = max(w_vec - cw, 0); 
    % TODO 
    % insufficient overlap penalty -- think about when this would happen opti wise 
    l_ol_penalty = max(l_ol_req - l_cs_vec, 0); 


    
    penalty = [force_order_penalty;...
                 neg_f_penalty; 
                 neg_L_penalty;...
                 neg_l_cs_penalty;... 
                 w_large_penalty;...
                 l_ol_penalty]; 


end 

function control_score = control_fitness(x, fp)

    % no penaltiues to incorporate here 
    % TODO -- actually run over all the fps but for now just the first 



    forceProfile = fp{1};   % TODO -- run over all of them 


    [f_vec, k_vec, L_vec, l_cs_vec] = split_design(x); % function to distribute 
    n = length(f_vec); 

    % NOTE -- we will need good way to return the control sequence itself because 
    % we will need to differentiate with respect to it 


    lims = [-100, 100];     % ridiculous but dont care about compatibility with this right now 

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


    % need varialbe for number of steps that we can normalize by 

    domRatio = inf; % 10000; % 
    numControl = 70;        % shouldn=t really need this -- abstract hese things out 
    numKeep = 16; 

    [bestSols, forces, minvals, varargout]  =  BQBF(test_actuator,...
                                            forceProfile, numKeep, numControl, domRatio) ; 

    %{
    U = bestSols{1}; 

    % TODO NEED TO ADD A WRAPPER AROUND BQBF TO ABSTRACT THIS NONSE OUT AND JUST RETURN U AND F 

    f_control = forces{1}; 
    t = forceProfile.time; 
    f_des = forceProfile.force; 
    %}


    control_score = minvals(1)/numControl; 
end 














