
%*********************************

% fp -- structure of force profiles 
% n = number of springs 
% w = weight on control (1 - w is weight on design)
% spingLims -- bounds on what dimensions springs can reasonably have 

% TODO -- add in options to use the hybrid methods 
% TODO -- add i/o options for tracking results 


%*********************************

% could add optActuator as an output but probs wont do anything with it in the short term 

function [fitness, data, genData] = actuator_optimization(n, fp, w, springLims)     

% TODO -- add a parser -- needs to specify cma type (hybrid or not plus maybe some other params )    

% NOTE flags/options for radial design? 

% NOTE -- an input parser could be really nice here 
% TODO -- assertion check on 0 <= w <= 1

k1 = ClutchSpringConstants.k1;
k2 = ClutchSpringConstants.k2;
cw = ClutchSpringConstants.cw; 
cf = ClutchSpringConstants.cf; 
t = springLims.t_max; 


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
% fitnessfun = @(x) w * design_fitness(x, springLims, maxDisp, minDisp) + (1 - w) * control_fitness(x, fp); 


fitnessfun = @(x) fitnessFunction(x, springLims, maxDisp, minDisp, fp, w);

disp('Playing with some function handles here......')

display(w_init); 
% Run the CMA -- get the results out an return 

% TODO 
cmaOptions.MaxGenerations = 40; 
cmaOptions.MaxStallGenerations = 10;   % 80 
parmdiffs = 0.05 * init_design;     % or keep at identity? idk 


% TODO -- add in oother cma options and maybe an arg parser for doing lamarck/baldwinian 
% just looking at where the issues arrise on the initial point 
%design_fitness(init_design, springLims, maxDisp, minDisp); 
%return 

% NOTE: would like to be able to initialize at a feasible point 

[~, init_penalty] = penalty(init_design, springLims, maxDisp, minDisp);   display(init_penalty); 

% [final_design, fullCost] = cma(init_design, fitnessfun, cmaOptions, 'init_scale', parmdiffs, 'hybrid', 'lamarckian') %  'init_scale', parmdiffs, 'print', true); 


hybridFun = @(x, data) deal(0, x);       % TODO -- update 


hybridFun = @(x,y) fixed_control_fitness(x, y, springLims, maxDisp, minDisp, fp, w);


% load('delete_me.mat')


%[final_design, fullCost, reason, dataHandles, genData] = cma(init_design, fitnessfun, cmaOptions, 'init_scale', parmdiffs,....
%                                                                 'hybrid', 'lamarckian', 'hybridFun', hybridFun); %  'init_scale', parmdiffs, 'print', true); 



%[final_design, fullCost, reason, dataHandles, genData] = cma(init_design, fitnessfun, cmaOptions, 'init_scale', parmdiffs,....
%                                                                 'hybrid', 'baldwinian', 'hybridFun', hybridFun); %  'init_scale', parmdiffs, 'print', true); 

[final_design, fullCost, reason, dataHandles, genData] = cma(init_design, fitnessfun, cmaOptions, 'init_scale', parmdiffs); %  'init_scale', parmdiffs, 'print', true); 


%[final_design, fullCost] = cma(init_design, fitnessfun, cmaOptions, 'init_scale', parmdiffs, 'print', true); 

% some prints 

des_final = design_fitness(final_design, springLims, maxDisp, minDisp) ;
control_final = control_fitness(final_design, fp); 

display(des_final)
display(control_final) 

[f_vec, k_vec, L_vec, l_cs_vec] = split_design(final_design); 

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

[~, final_penalty_vec] = penalty(final_design, springLims, maxDisp, minDisp); 
display(final_penalty_vec)

%% TODO -- probably move these out to their own places 


%
% ----- Assign Outputs 
% 
data = struct(); 
%data.control_cost = 
%data.design_cost = 
%data.design_penalty = 

fitness = fullCost;  


% NOTE -- genData doesnt really need to go out but it will help with debugging 



%{
% playing around with local descent in this space i guess??????


gen = genData{end};     % take the data from the most recent generation 
num_samples = length(gen.fitness); 
init_samples = gen.samples;         % want to see if we can improve on these 
shifted_samples = 0 * init_samples;     % will fill this up 

for k = 1:num_samples

    disp(['-------------------------- sample ', num2str(k), '---------------------']); 

    % Do a sanity check on the fitness score -- make sure our "cost function" here lines up -- (actually ignore for now -- just seeing if we can make improvments )
    % may need to NOT use mids to make it all line up but there could also be issues with bqbf (could not line up but be close and we still check for improvements)
    
    cur_sample = init_samples(:, k); 
    %display(cur_sample)

    init_fitness = gen.fitness(k);

    init_design_score = design_fitness(cur_sample, springLims, maxDisp, minDisp);
    init_penalty_score = penalty(cur_sample, springLims, maxDisp, minDisp);

    display(init_design_score)
    display(init_penalty_score)
    display(init_fitness)

    %U = gen.data{k}.U;
    %X = gen.data{k}.X;
    %f_des = gen.data{k}.f_des; 
    fit_data = gen.data{k}; 

    % Could add in other "data" including the cost breakdown to make debug easier 
    [new_fitness_check, new_fitness] = fixed_control_fitness(cur_sample, fit_data, springLims, maxDisp, minDisp, fp, w); 

    new_fitness = fitnessfun(new_sample);   % to acually do this 
    new_design_score = design_fitness(new_sample, springLims, maxDisp, minDisp);
    new_penalty_score = penalty(new_sample, springLims, maxDisp, minDisp);


    display(new_design_score)
    display(new_penalty_score)
    display(new_fitness)            % compute from the full function 
    display(new_fitness_check) 

    % now we will do some stuff to shift it

    % Add a very small quadratic penalty for veering too far away from initial point -- 

    % local_descent(cur_sample)       % This will ultimately require lots of other inputs so cost functions can actually be applied 
end
%}

end 



%%=======================================================================================
%
%                                       FUNCTION DEFS 
%
%=======================================================================================



function change_cost = change_penalty(new_sample, init_sample)

    % lets limit ourselves to 10% change from the initial point -- effectively a weighted L1 norm 

    allowable_diff = 0.05 * init_sample; 

    abs_change = abs(new_sample - init_sample); 

    penalty_vec = min(allowable_diff - abs_change, 0);

    rho = 8000; % could add penalty increasing in CMA     
    p_quad = norm(penalty_vec, 2)^2;  
    p_count = nnz(penalty_vec); 
    change_cost = p_count + rho*p_quad;

end 

function [new_fitness, new_sample] = fixed_control_fitness(init_sample, fit_data, springLims, maxDisp, minDisp, fp,  w) 

    % TODO -- for lamarckian learning especially, may need to limit how for we move from initial point -- could be in the form of small quadratic cost

    n = round(length(init_sample)/4); 
    design_cost_fun = @(x) design_fitness(x, springLims, maxDisp, minDisp) ;
    penalty_cost_fun = @(x) penalty(x, springLims, maxDisp, minDisp);       % TODO -- maybe plus additionally quadratic on how far we moved! 

    change_cost_fun = @(x) change_penalty(x, init_sample);


    % NOTE -- may want to do that penalty seperately so can see if actually improving without it 

    % distribute the fit data 
    U = fit_data.U; 
    X = fit_data.X;
    f_des = fit_data.f_des; 

    % Now define the control cost assuming fixed control strategy 
    % [f_vec, k_vec, L_vec, l_cs_vec] = split_design(x);

    dt =  fp{1}.time(2) - fp{1}.time(1);


    control_cost_fun = @(x) dt * (norm(f_des - (((x(1:n))'*U) + (x(n+1:2*n))'*(U .* X))', 2)^2)/length(f_des);      % should be same as force returned by bqbf  -- TODO will need to check if norm is squared there or not! 

    
    total_cost_fun = @(x) penalty_cost_fun(x) + w*design_cost_fun(x) + (1-w) * control_cost_fun(x) + change_cost_fun(x);




    % VAGUE SANITY CHECK ON INIT VALUES 

    cmaOptions.MaxGenerations = 300; 
    cmaOptions.MaxStallGenerations = 50;   % 80 
    parmdiffs = 0.05 * init_sample;     % or keep at identity? idk 

    %disp('-Running Local Steps -');

    [new_sample, new_fitness, reason] =  cma(init_sample, total_cost_fun, cmaOptions, 'init_scale', parmdiffs, 'print', false);

    %{
    f_out_init = (((init_sample(1:n))'*U) + (init_sample(n+1:2*n))'*(U .* X));
    f_out_new = (((new_sample(1:n))'*U) + (new_sample(n+1:2*n))'*(U .* X));
    figure, 
    time = fp{1}.time; 
    plot(time, fp{1}.force, 'k--'), hold on 
    plot(time(:), f_out_init(:), 'b');
    plot(time(:), f_out_new(:), 'r');
    hold off 
    %} 

    % TODO IN THE MORNING -- Break down before and after 
        % design score  
end 





function  [cost, data]  = fitnessFunction(x, springLims, maxDisp, minDisp, fp, w)       % For main opti 

    design_cost = design_fitness(x, springLims, maxDisp, minDisp) ;
    
    [control_cost, c_data] =  control_fitness(x, fp); 
    
    penalty_cost = penalty(x, springLims, maxDisp, minDisp);

    data = c_data;
    
    data.penalty_cost = penalty_cost; 
    data.design_cost = design_cost; 
    data.control_cost = control_cost; 

    cost =  penalty_cost + w * design_cost + (1 - w) *control_cost; 
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

    % NOTE max disp is a scalar here 
    l0_vec = (f_vec./k_vec) - (k2/k1)*L_vec; 
    tops = l0_vec + maxDisp + l_cs_vec;
    F_max_vec = f_vec + (maxDisp .* k_vec); 
    l_ol_req = F_max_vec./(cw * cf);    % this is a REQUIRED l_ol 

    % NOTE 2 lines below becomes dfferent for rotary actuator 
    l_co_vec = max(max(tops) - (l0_vec + minDisp + l_cs_vec - l_ol_req)) * ones(n, 1); 
    bottoms = min(tops - l_co_vec, 0); 
    design_score = max(max(tops) - min(bottoms));    % effective length 

end 


function [penalty_cost, penalty_vec] = penalty(x, springLims, maxDisp, minDisp)

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
    
    penalty_vec = [force_order_penalty;...
                 neg_f_penalty; 
                 neg_L_penalty;...
                 neg_l_cs_penalty;... 
                 w_large_penalty;...
                 l_ol_penalty]; 

    % then take quadratic and counts function on the penalty 
    rho = 5000; % could add penalty increasing in CMA     
    p_quad = norm(penalty_vec, 2)^2;  
    p_count = nnz(penalty_vec); 
    penalty_cost = p_count + rho*p_quad;
end 

function [control_score, control_data] = control_fitness(x, fp)

    % no penaltiues to incorporate here 
    % TODO -- actually run over all the fps but for now just the first 


    forceProfile = fp{1};   % TODO -- run over all of them 


    [f_vec, k_vec, L_vec, l_cs_vec] = split_design(x); % function to distribute 
    n = length(f_vec); 

    % NOTE -- we will need good way to return the control sequence itself because 
    % we will need to differentiate with respect to it 


    lims = [-100, 100];     % ridiculous but dont care about compatibility with this right now 

    %test_actuator = Actuator(n, 'vector', f_vec, 'S', k_vec,...
    %                                'limits', lims, 'kp', 1e-5, 'kc', 1e-5); 



    test_actuator = Actuator(n, 'vector', f_vec, 'S', k_vec,...
                                    'limits', lims, 'kp', 1e-8, 'kc', 1e-8); 
                                    
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


    % TODO -- string together U and X values for when we have multiple force profiles 
    % -- fast way is it initilialize them 


    f_des = forceProfile.force(:); 
    f_des_mid = (f_des(1:end-1) + f_des(2:end))/2; 


    U = bestSols{1}; 


    Up = U(:, 1:end-1); % U priome 
    d = diff(forceProfile.disp)';   % col vec
    [~, h] = size(Up); 
    D = triu(repmat(d, [1, h])); % be careful with row vec vs col v 

    % NEED TO INCLUDE THE x0 term here -- because really x0 
    x0 = zeros(n, 1);
    X = [x0, Up*D];        % need to account for an initial position too ! 

    X_mid = 0.5 * (X(:, 1:end-1) + X(:, 2:end)); 


    % 
    control_data = struct(); 
    control_data.Up = Up; 
    control_data.X_mid= X_mid; 
    control_data.f_des_mid = f_des_mid;    % MIDs!!!!


    control_data.f_des = f_des; 
    control_data.U = U; 
    control_data.X = X; 


    control_score = minvals(1)/numControl; 
end 














