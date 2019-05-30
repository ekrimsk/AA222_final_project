%%******************************************************************
%	Energy Recycling Actuator Design Optimization
%	AA222 Final Project Main Script, Spring 2019
%
%	Erez Krimsky 
%   ekrimsky@stanford.edu
%
%
% 	Script for comparing convergence using different CMA types 
%
%*******************************************************************

%
close all; clear all; clc; 

% make sure parraelization is running 


% set a randomization seed -- will proably need to run each multiple times

% could throw multiple curves on the same plot 
%\

%{


% Lets set the same weight for each 
w = 0.4; 
n = 4; 			% NOTE Could loop through a few later if we have the time 
num_opts = 1; 


figure, hold on 
for j = 1:num_opts



	% Run the Basic CMA-Approach 
	disp(' Running Basic Opti');
	rng(5);		% ressetting the random number generator 
	[fitness, optimal_design,  data, genData] = actuator_optimization_wrapper(n, w, 'none');
	basicGenData = genData; 
	[means, basic_mins, maxs, stds] = process_gen_data(basicGenData); 



	disp(' Running Baldwinian Opti')
	rng(5);		% ressetting the random number generator 
	[fitness, optimal_design,  data, genData] = actuator_optimization_wrapper(n, w, 'baldwinian');
	baldwinianGenData = genData; 
	[means, baldwinian_mins, maxs, stds] = process_gen_data(baldwinianGenData); 



	disp(' Running Lamarckian Opti')
	rng(5);		% ressetting the random number generator 
	[fitness, optimal_design,  data, genData] = actuator_optimization_wrapper(n, w, 'lamarckian');
	lamarckianGenData = genData; 
	[means, lamarckian_mins, maxs, stds] = process_gen_data(lamarckianGenData); 





	% Then post process the genData after looking through how its structured 


	% Should only plot means really 

	% Lets Do for w = 0.5, W = 0.1, W = 0.9 for now.... 


	% Loop through processing the data 

	
	%plot(gens, means,'k', 'DisplayName', 'mean'), hold on 
	%plot(gens, maxs, 'g', 'DisplayName', 'max')
	%plot(1:length(basic_mins), basic_mins, 'r', 'DisplayName', 'Basic Min');
	%plot(1:length(baldwinian_mins), baldwinian_mins, 'g', 'DisplayName', 'Baldwinian Min');
	%plot(1:length(lamarckian_mins), lamarckian_mins, 'b', 'DisplayName', 'Lamarckian Min');


	% fill_in_between(gens, mins, maxs); 
	% not sure if should incorporate stds 

end 


save('cma_comparison.mat')

	% shade = fill_in_between(x1, y1, y2); 

%}



 load('cma_comparison.mat')


genData = baldwinianGenData;


[baldwinian_pre_mins, baldwinian_init_mins] = process_hybrid_gen(baldwinianGenData);
[lamarckian_pre_mins, lamarckian_init_mins] = process_hybrid_gen(lamarckianGenData);


figure, hold on 

	plot(1:length(basic_mins), basic_mins, 'r', 'DisplayName', 'Basic Min');


	plot(1:length(baldwinian_mins), baldwinian_mins, 'g', 'DisplayName', 'Baldwinian Min');
	%plot(1:length(baldwinian_pre_mins), baldwinian_pre_mins, 'k', 'DisplayName', 'Baldwinian Pre Min');
	plot(1:length(baldwinian_init_mins), baldwinian_init_mins, 'g--', 'DisplayName', 'Baldwinian init Min');


	plot(1:length(lamarckian_mins), lamarckian_mins, 'b', 'DisplayName', 'Lamarckian Min');
	%plot(1:length(lamarckian_pre_mins), lamarckian_pre_mins, 'y', 'DisplayName', 'Lamarckian Pre Min');
	plot(1:length(lamarckian_init_mins), lamarckian_init_mins, 'b--', 'DisplayName', 'Lamarckian init Min');



	%plot(1:length(lamarckian_mins), lamarckian_mins, 'b', 'DisplayName', 'Lamarckian Min');
hold off 	

print('hybrid_converge_compare', '-dpng', '-r300'); 


function [pre_mins, init_mins] = process_hybrid_gen(genData)
	
	numGens = numel(genData);

	pre_mins = zeros(numGens, 1);       % NOTE these are what came before our mins -- NOT the same as the mins of the init pop 

	init_mins = zeros(numGens, 1); 		% mins of the init population -- not the same as what lead to new min per se

	for i = 1:numGens 
		[minFit, minIdx] = min(genData{i}.fitness);		% post hybrid fitness 

		pre_mins(i) = genData{i}.data{minIdx}.initial_fitness; 	% can use w to back calculate out if we need 

		% 
		pop_size = numel(genData{i}.data);
		init_pop_fit = zeros(pop_size, 1);
		for k = 1:pop_size
			init_pop_fit(k) = genData{i}.data{k}.initial_fitness; 
		end 

		init_mins(i) = min(init_pop_fit); 
	end 

end 	





function plot_handle = fill_in_between(x, y1, y2)
	% https://www.mathworks.com/matlabcentral/answers/180829-shade-area-between-graphs
	x2 = [x, fliplr(x)];
	inBetween = [y1, fliplr(y2)]
	plot_handle = fill(x2, inBetween, 'g');

	plot_handle.FaceAlpha = 0.5;
	plot_handle.LineStyle = 'none'; 
end 


