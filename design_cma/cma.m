function [xbest, fitness, reason, dataHandles, genData] = cma(xguess, fitnessfun,...
                                                             options, varargin)   
%*******************************************************************************
%   Function:
%       CMA.m
%   
%   Description:
%       Base code copied originally from wikipedia for mu/lambda CMA es.
%        
%
%   Inputs:
%       xguess - inital guess for solution vector. Use all ones or zeros        
%               if no information about the problem is known%
%       fitnessfun - handle to the function for evalauating fitness of 
%                   the solution. Lower fitness means better performance
%       options - structure with fields "MaxGenerations" and
%                   "MaxStallGenerations" which dictate stopping criteria
%       varargin{1} - dimensional scaling vector for when parameters of 
%                       the input space are on different scales
%       varargin{2} - handle to a constraining function that caps the CMA
%                       guesses at defined constraints when solving a 
%                       constrained problem.
%       varargin{3} - true (print results) or false (dont print) 
%       varargin{4} - handle of figure to plot on 
%       varargin{5} - handle to function to be called for updating plot
%       varargin{6} - name of video to create 
%
%
%   Outputs:
%       xbest - the best guess for the solution 
%       fitness - fitness value for best solution
%       reason - reason for termination (see end of code for key)
%
%   Revisions:
%       4/02/18 - changed parmdiffs scaling 
%       4/03/18 - parmdiffs check for zero entries
%       4/26/18 - added parallelization
%       4/27/18 - improved parallization
%       6/12/18 - adds index for best value as argument into plot 
%       6/19/18 - changed to ffmpeg script for creating animations

%       AA222 Modifications (May 2019)
%           - Removed constraining function input 
%           - Removed all positional varargin and replaced with an input parser 
%
%  
%
%   Author: 
%       Erez Krimsky, ekrimsky@stanford.edu, 2/19/18 
%       Stanford University, Biomechatronics Lab 
%******************************************************************************
N = numel(xguess);



% parse the inputs 

% required inputs -- the first thress 
p = inputParser; 

defaultParmdiffs =  ones(N, 1);   
defaultMovieName = '';  % no movie 


addParameter(p, 'init_scale', defaultParmdiffs, @isnumeric)
addParameter(p, 'plot', false, @islogical);
addParameter(p, 'plotFun', 0); 
addParameter(p, 'print', true, @islogical);
addParameter(p, 'figHandle', 0);    % could make handle to empty object and then do typechecking on this 




addParameter(p, 'movieName', defaultMovieName);    % TODO -- add this functionality if have time 

defaultLearning = 'none';
expectedLearning = {'none','lamarckian','baldwinian'};
addParameter(p, 'hybrid',defaultLearning,  @(x) any(validatestring(x, expectedLearning)));


addParameter(p, 'hybridFun', 0);    % actual function to call in hybrid method, could add some validation 





parse(p, varargin{:}); 
% display(p.Results)

parmdiffs = p.Results.init_scale; 
parmdiffs(parmdiffs == 0) = 1; % if zero, will kill opti 

doprints = p.Results.print; 
hybrid = p.Results.hybrid;      % default is none 
hybridFun = p.Results.hybridFun; 


movieName = p.Results.movieName; 
makeMovie = false;
if ~strcmp(movieName, defaultMovieName)
    makeMovie = true;
end 


dataHandles = {}; 
doplots = false; 


xmean = reshape(xguess, N, 1);
% parmdiffs = ones(N, 1); % will change if there is is a constraining function

% TODO -- add plotting functionality back in 
% For creating plots 
%if nargin > 7
%    fig = varargin{4};
%    plotfun = varargin{5}; 
%    doplots = true; 
%end 

% For creating an animation of the plots 

% TODO -- add movie functionality back in 
%makeMovie = false;
%if nargin > 8
%    movieName = varargin{6};
%   makeMovie = true;
%    dir1 = 'cma_frames';
%    mkdir(dir1); 
%    system(['rm ', dir1, '/*']);
%    F(options.MaxGenerations) = struct('cdata',[],'colormap',[]);
%end 



%% ----------------------- Stopping Criteria --------------------------- 
dRatio = 1e8; % (max(D) > dRatio * min(D))
normRatio = 1e-6; %(maxnorm/norm(xmean, 2) < normRatio) 
stopfitness = 1e-5; % (arfitness(1) <= stopfitness)
stopeval = 1e6*N^2;   % stop after stopeval number of function evaluations
plateauGens = 60; % number of gens with very similar values we stop at 
plateauCov = 1e-6/N; % cov to stop at (of last plateau gens # of gens)
lastGens = nan(1, plateauGens); % for storing previous mean fitnesses 


%% --------------------  Initialization -------------------------------- 
maxIter = options.MaxGenerations; 
maxStall = options.MaxStallGenerations; % generations without an improvement
minVal = inf; 
lastImproveStep = 0; 

% User defined input parameters (need to be edited)
sigma = 0.3;


% Strategy parameter setting: Selection  
lambda = 4+floor(3*log(N));  % population size, offspring number 



mu = lambda/2;               % number of parents/points for recombination
weights = log(mu+1/2)-log(1:mu)'; % muXone array for weighted recombination
mu = floor(mu);        
weights = weights/sum(weights);     % normalize recombination weights array
mueff=sum(weights)^2/sum(weights.^2); % variance-effectiveness of sum w_i x_i

% Strategy parameter setting: Adaptation
cc = (4+mueff/N) / (N+4 + 2*mueff/N);  % time constant for cumulation for C

cs = (mueff+2) / (N+mueff+5);  % t-const for cumulation for sigma control

c1 = 2 / ((N+1.3)^2+mueff);    % learning rate for rank-one update of C
cmu = min(1-c1, 2 * (mueff-2+1/mueff) / ((N+2)^2+mueff));  % and for rank-mu update


damps = 1 + 2*max(0, sqrt((mueff-1)/(N+1))-1) + cs; % damping for sigma 
                                                    % usually close to 1
% Initialize dynamic (internal) strategy parameters and constants
pc = zeros(N,1); ps = zeros(N,1);   % evolution paths for C and sigma
B = eye(N,N);                       % B defines the coordinate system

%  D = ones(N,1);                      % diagonal D defines the scaling
D = reshape(parmdiffs, N, 1);         % My version  -- This Actually matters a ton!!

C = B * diag(D.^2) * B';            % covariance matrix C
invsqrtC = B * diag(D.^-1) * B';    % C^-1/2 
eigeneval = 0;                      % track update of B and D
chiN=N^0.5*(1-1/(4*N)+1/(21*N^2));  % expectation of 
                                    %   ||N(0,I)|| == norm(randn(N,1)) 
% -------------------- Generation Loop --------------------------------
counteval = 0;  % the next 40 lines contain the 20 lines of interesting code 
iter = 1; 

genData = {};

while (counteval < stopeval) && (iter <= maxIter) && ...
                                        (iter - lastImproveStep < maxStall)
  
    % Generate and evaluate lambda offspring
    constrained = zeros(lambda, 1); % number of samples that have been constrained
    

    popStats = struct();    % data for members of population for one generation 
    fit_data = cell(lambda, 1);     % 1 cell per pop member 

    parfor k = 1:lambda,
        newsample = xmean + sigma * (B * (D .* randn(N,1))); % m + sig * Normal(0,C) 

        if ( strcmp(hybrid, 'lamarckian') || strcmp(hybrid, 'baldwinian'))   
    
            [sample_fitness, sample_fit_data] = fitnessfun(newsample);

            if (sample_fit_data.penalty_cost > 1)
                % if the initial thing had penalties, ignore it 
                updated_fitness = sample_fitness; 
                updated_sample = newsample; 

            else 
                [updated_fitness, updated_sample] = hybridFun(newsample, sample_fit_data); 
            end     

           


            % MIGHT BE USEFUL TO DISREGARD ANY UPDTE THAT DOESNT HELP!

            if (updated_fitness > sample_fitness)
                display('updated fitness worse')
                display(sample_fitness)
                display(updated_fitness)
            end 

            
            % Just some metrics 
            update_diff = newsample - updated_sample;
            pct_change = 100 * (update_diff./newsample);

            
            %{
      
            sample_fit_data.inital_sample = newsample;
            sample_fit_data.updated_sample = updated_sample;

            sampe_fit_data.initial_fitness = sample_fitness; 
            sample_fit_data.new_fitness = updated_fitness; 

            %}


            fit_data{k} = sample_fit_data;      % Constains fitness data from the ORIGINAL opti point 
            fit_data{k}.initial_sample = newsample; 
            fit_data{k}.updated_sample = updated_sample; 

            fit_data{k}.initial_fitness = sample_fitness;
            fit_data{k}.new_fitness = updated_fitness; 
            
            sample_fitness = updated_fitness; 
            if (strcmp(hybrid, 'lamarckian'))
                newsample = updated_sample;     % ---  just skip this for baldwinian 
            end 


            

            % Adding some of the update information to sample_fit_data so we can process later 


        else 
            sample_fitness = fitnessfun(newsample);     % may return the other data??? 
        end 
        
        arfitness(k) = sample_fitness;
        arx(:,k) = newsample;


        % want to log all the samples and thier fitness and any other interesting data to process 
    end
    popStats.samples = arx; 
    popStats.fitness = arfitness;

    % NOTE -- want to dump this info even for the default case 
    if (strcmp(hybrid, 'lamarckian'))   
        popStats.data = fit_data; 
    elseif (strcmp(hybrid, 'baldwinian'))   
        popStats.data = fit_data; 
    end 



    genData{end + 1} = popStats; 

    counteval = counteval+lambda;  
  
    % Sort by fitness and compute weighted mean into xmean
    [arfitness, arindex] = sort(arfitness); % minimization
    minfitness = arfitness(1); 

    if (minfitness < minVal)
      minVal = minfitness;
      lastImproveStep = iter; 
    end 

    xold = xmean;
    xmean = arx(:,arindex(1:mu))*weights;   % recombination, new mean value
  
    % Cumulation: Update evolution paths
    ps = (1-cs)*ps ... 
          + sqrt(cs*(2-cs)*mueff) * invsqrtC * (xmean-xold) / sigma; 
    hsig = norm(ps)/sqrt(1-(1-cs)^(2*counteval/lambda))/chiN < 1.4 + 2/(N+1);
    pc = (1-cc)*pc ...
          + hsig * sqrt(cc*(2-cc)*mueff) * (xmean-xold) / sigma;

    % Adapt covariance matrix C
    artmp = (1/sigma) * (arx(:,arindex(1:mu))-repmat(xold,1,mu));



    C = (1-c1-cmu) * C ...                  % regard old matrix  
         + c1 * (pc*pc' ...                 % plus rank one update
                 + (1-hsig) * cc*(2-cc) * C) ... % minor correction if hsig==0
         + cmu * artmp * diag(weights) * artmp'; % plus rank mu update

    % Adapt step size sigma
    sigma = sigma * exp((cs/damps)*(norm(ps)/chiN - 1)); 
  
    % Decomposition of C into B*diag(D.^2)*B' (diagonalization)
    if counteval - eigeneval > lambda/(c1+cmu)/N/10  % to achieve O(N^2)
        eigeneval = counteval;
        C = triu(C) + triu(C,1)'; % enforce symmetry
        [B,D] = eig(C);           % eigen decomposition, B==normalized eigenvectors
        D = sqrt(diag(D));        % D is a vector of standard deviations now
        invsqrtC = B * diag(D.^-1) * B';
    end
  


    if doplots
        [minFit, minIdx] = min(arfitness);
        if iter == 1
            set(0, 'CurrentFigure', fig); hold on 
            nullcell = {}; % no previous things to update
            dataHandles = plotfun(fig, arx, nullcell, iter, minFit, arindex(1));
        else
            dataHandles = plotfun(fig, arx, dataHandles, iter, minFit, arindex(1));
        end 

        if makeMovie
            framestr = sprintf('%s/frame%0.5i', dir1, iter);
            print(framestr, '-dpng', '-r300');
            F(iter) = getframe(gcf);  
        end 
    end 

    % Check how far the farthest vector is from the mean (in terms of 2-norm)
    % normalize this and use it as another stopping criteria 
    % EREZ added this critera 
    maxnorm = 0; 
    for i = 1:lambda
        maxnorm = max(maxnorm, norm(xmean - arx(:, i), 2));
    end 
    
    meanfit = mean(arfitness(~isinf(arfitness))); 
    if doprints
        constrainedCount = nnz(constrained);
        fprintf('Iteration: %i, Mean fitness: %0.4f, Max fitness: %0.4f, Min fitness: %0.4f\n',...
                                              iter, meanfit, max(arfitness), min(arfitness));
    end

    % check if plateuad 
    lastGens(2:end) = lastGens(1:end - 1);
    lastGens(1) = meanfit;


    % Break, if fitness is good enough or condition exceeds 1e14, better 
    % termination methods are advisable 
    if (arfitness(1) <= stopfitness) || (maxnorm/norm(xmean, 2) < normRatio) ...
                                      || (max(D) > dRatio * min(D)) ... %(max(D) > 1e7 * min(D))
                                      || (std(lastGens)/mean(lastGens) < plateauCov)
        break;
    end
    iter = iter + 1; 

    
end % while, end generation loop

xbest = arx(:, arindex(1)); % Return best point of last iteration.
                           % Notice that xmean is expected to be even
                           % better.
fitness = arfitness(1); 

reason = -1; % reason for termination
if (arfitness(1) <= stopfitness)
    reason = 1; 
    reasonStr = sprintf('Optimization terminated: min fitness reached\n');
elseif (maxnorm/norm(xmean, 2) < normRatio) 
    reason = 2; 
    reasonStr = sprintf('Optimization terminated: min variation reached \n');
elseif (max(D) > dRatio * min(D))
    reason = 3; 
    reasonStr = sprintf('Optimization terminated: D scale limit reached \n');
elseif (iter >= maxIter)
    reason = 4; 
    reasonStr = sprintf('Optimization terminated: maximum iterations reached\n');
elseif ((iter - lastImproveStep) == maxStall)
    reason = 5; 
    reasonStr = sprintf(['Optimization terminated: maximum ',...
                                            'stall generations reached\n']);
elseif (counteval == stopeval)
    reason = 6; 
    reasonStr = sprintf('Optimization terminated: maximum function evaluations\n');
elseif (std(lastGens)/mean(lastGens) < plateauCov)
    reason = 7; 
    reasonStr = sprintf('Optimization terminated: solutions reached a plateau\n');
end 

if doprints
    fprintf('%s', reasonStr);
end 

%% ------------------------ Create movie from figures -------------------------
if makeMovie
    fprintf('\nCreating movie: %s\n', movieName);
    framerate = 6;
    % call an ffmpeg script 
    system(['ffmpeg -y -r ', num2str(framerate),...
            ' -start_number 1 -i ', dir1, '/frame%05d.png -vcodec mpeg4 ',...
             movieName, '.avi']);
end 
