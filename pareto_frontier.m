%%******************************************************************
%	Generate Pareto Front for the problem using CMA-ES based 
%	optimizer.
%
%	AA222 Final Project, Spring 2019
%	Erez Krimsky 
%   ekrimsky@stanford.edu
%*******************************************************************
close all; clear all; clc;

num_check = 15; 	% NOTE -- will increase to make plots later 

weight_vec = linspace(0, 1, num_check); 

design_scores = zeros(num_check, 1); 
control_scores = zeros(num_check, 1); 



% We will loop through lots of n values so we can make plot understanding how the pareto front changes 


% For now n = 4, could throw another loop around this if we want 

%n_vec = [3, 4, 5, 6, 7];
n_vec = [3, 4, 5, 6, 7];

design_scores_list = cell(length(n_vec), 1);
control_scores_list = cell(length(n_vec), 1);

number_of_opts = length(n_vec)*num_check; 

fprintf('Number of optimizations to run: %d...\n', number_of_opts)

for j = 1:length(n_vec)
	n = n_vec(j); 
	for k = 1:num_check
		w = weight_vec(k); 
		[fitness, data, genData] = actuator_optimization_wrapper(n, w, 'none');
		design_scores(k) = fitness/w;
		control_scores(k) = fitness/(1 - w); 
	end 
	design_scores_list{j} = design_scores;
	control_scores_list{j} = control_scores;
end 
save('pareto_data.mat'); 

%% Create the figure 
figure, hold all
for j = 1:length(n_vec)
	n = n_vec(j); 
	design_scores = design_scores_list{j};
	control_scores = control_scores_list{j};
	plot(control_scores, design_scores, 'x--', 'Displayname', sprintf('n = %d', n));
end 



ylabel('Actuator Length')
xlabel('Actuator Control Cost')
print('pareto_frontier', '-dpng', '-r300');
hold off 

