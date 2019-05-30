%%******************************************************************
%   Energy Recycling Actuator Design Optimization
%   AA222 Final Project Main Script, Spring 2019
%
%   Erez Krimsky 
%   ekrimsky@stanford.edu
%
%
%   Script for comparing convergence using different CMA types 
%
%*******************************************************************


close all; clear all; clc; 

% make sure parraelization is running 


% set a randomization seed -- will proably need to run each multiple times

% could throw multiple curves on the same plot 
%\



% Lets set the same weight for each 
w = 0.5; 
n_list = [3, 4, 5, 6, 7];         % TODO -- add more later 
min_lists = cell(length(n_list), 1); 

for i = 1:length(n_list);

    n = n_list(i); 



        % Run the Basic CMA-Approach 
    disp(' Running Basic Opti');
    rng(5);     % ressetting the random number generator 
    [fitness, optimal_design,  data, genData] = actuator_optimization_wrapper(n, w, 'none');
    basicGenData = genData; 
    [means, basic_mins, maxs, stds] = process_gen_data(basicGenData); 
    min_lists{i} = basic_mins; 

end 

save('basic_cma_convergence.mat'); 


load('basic_cma_convergence.mat'); 


fig = figure; hold all 
for i = 1:length(n_list);
    n = n_list(i); 
    mins = min_lists{i}; 
    plot(1:length(mins), mins, 'Displayname', sprintf('n = %d', n)); 

end 

leg = legend;
set(leg, 'Interpreter', 'latex')

xlabel('Generations');
ylabel('Cost')

print('basic_convergence_plot', '-dpng', '-r300');







