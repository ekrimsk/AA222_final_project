

close all; clear all; clc; 

% For comparing the control before hybrid optimization and after 



 load('hybrid_compare_input.mat');      % this was orignally created as a copy of cma_comparison.mat 




% will pull out some generation from this look at the results before and then the post results from applying the hybird step 





genData = baldwinianGenData; 

% arbitrary choose gen 100 
thisGen = genData{1};


[min_fit, min_idx] = max(thisGen.fitness); 

data = thisGen.data{min_idx}; 


n = length(data.initial_sample)/4; 

initial_sample = data.initial_sample; 
updated_sample = data.updated_sample; 

f_des = data.f_des; 

b_init = initial_sample(1:n);
k_init = initial_sample((n + 1):(2*n));

b_new = updated_sample(1:n);
k_new = updated_sample((n + 1):(2*n));


% copying code from figure script 


U = data.U; 
X = data.X; 


f_init =  (b_init'*U) + k_init'*(U .* X);      % should be same as force returned by bqbf 


f_new =  (b_new'*U) + k_new'*(U .* X);      % should be same as force returned by bqbf 


t = 1:length(f_new); 
t = t(:); 

figure, hold on  
plot(t, f_des, 'k')
plot(t, f_init, 'b')
plot(t, f_new, 'r')

% TODO -- make it look good   

hold off 