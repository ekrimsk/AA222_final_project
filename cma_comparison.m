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

close all; clear all; clc; 

% make sure parraelization is running 


% set a randomization seed -- will proably need to run each multiple times

% could throw multiple curves on the same plot 
%\



% Lets set the same weight for each 
w = 0.5; 
n = 4; 			% NOTE Could loop through a few later if we have the time 



figure, hold on 
for num_opts = 1:5



	% Run the Basic CMA-Approach 
	disp(' Running Basic Opti');
	rng(5);		% ressetting the random number generator 
	[fitness, optimal_design,  data, genData] = actuator_optimization_wrapper(n, w, 'none');
	basicGenData = genData; 


	disp(' Running Baldwinian Opti')
	rng(5);		% ressetting the random number generator 
	[fitness, optimal_design,  data, genData] = actuator_optimization_wrapper(n, w, 'baldwinian');
	baldwinianGenData = genData; 

	disp(' Running Lamarckian Opti')
	rng(5);		% ressetting the random number generator 
	[fitness, optimal_design,  data, genData] = actuator_optimization_wrapper(n, w, 'lamarckian');
	lamarckianGenData = genData; 


	save('cma_comparison.mat')


	% Then post process the genData after looking through how its structured 


	% Should only plot means really 

	% Lets Do for w = 0.5, W = 0.1, W = 0.9 for now.... 


	[means, basic_mins, maxs, stds] = process_gen_data(basicGenData); 
	[means, baldwinian_mins, maxs, stds] = process_gen_data(baldwinianGenData); 
	[means, lamarckian_mins, maxs, stds] = process_gen_data(lamarckianGenData); 
	% Want to make big convergence plot shoing lines for each --- 
	
	%plot(gens, means,'k', 'DisplayName', 'mean'), hold on 
	%plot(gens, maxs, 'g', 'DisplayName', 'max')
	plot(1:length(basic_mins), basic_mins, 'r', 'DisplayName', 'Basic Min');
	plot(1:length(baldwinian_mins), baldwinian_mins, 'g', 'DisplayName', 'Baldwinian Min');
	plot(1:length(lamarckian_mins), lamarckian_mins, 'b', 'DisplayName', 'Lamarckian Min');


	% fill_in_between(gens, mins, maxs); 
	% not sure if should incorporate stds 



end 


	legend show; 
	hold off 
% shade = fill_in_between(x1, y1, y2); 




function plot_handle = fill_in_between(x, y1, y2)
	% https://www.mathworks.com/matlabcentral/answers/180829-shade-area-between-graphs
	x2 = [x, fliplr(x)];
	inBetween = [y1, fliplr(y2)]
	plot_handle = fill(x2, inBetween, 'g');

	plot_handle.FaceAlpha = 0.5;
	plot_handle.LineStyle = 'none'; 
end 


function [means, mins, maxs, stds] = process_gen_data(genData); 
	numGens = numel(genData); 
	means = zeros(numGens, 1);
	mins = zeros(numGens, 1);
	maxs = zeros(numGens, 1);
	stds = zeros(numGens, 1);

	% NOTE - could add 'quartile' data 
	for k = 1:numGens
		fits = genData{k}.fitness;
		means(k) = mean(fits);
		mins(k) = min(fits);
		maxs(k) = max(fits);
		stds(k) = std(fits);
	end 
end 







