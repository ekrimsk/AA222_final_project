function [bestSols, forces, minvals, varargout]  =  BQBF(actuator,...
								 		forceProfile, numKeep, numControl,...
								 						  domRatio,  varargin) 
%******************************************************************************
%	Function:	
%		BQBF.m 
%
%	Description:
%		Runs a Branching Quasi-Brute Force optimization for determining
%		the control steps that should be taken for a given actuator 
%		and desired force profile. 
%
%	Inputs:
%		actuator - actuator object with properties to be replicated in 
%					the optimization of the control
%		forceProfile - structure with force profile information
%		numKeep - number of solutions to keep after each step 
%		numControl - number of control decisions to make, the first will be
%					at the initial time and the last at the final time  
%		domRatio - dominance ratio (use inf for no dominance)
				
%		varargin{1} - true or false for making plots while optimizing 
%		varargin{2} - number of solutions to put on the plot (will 
%						default to 4 if not specific)
%		varargin{3} - initial starting position for clutches specified 
% 						in a (n x 1) vector or 'false' to use default
%		varargin{4} - initial starting clutch status in (1 x n) vector.
%						Important for replanning. Use false for default
% 
% 		varargin{5} - true or false (true to create movie from plots)
%		varargin{6} - movie name for exported movie (if varargin{5} = true)
%
%	Outputs:
%		bestSols - cell array of control matrices for best solutions at the
%					final time step
%		minvals - list of scores for corresponding final solutions
%		varargout{1} - time in seconds to solve 
%		varargout{2} - handle to figure produced if plot was requested 
%
%	Revisions:
%		3/15/18 - added 'dt' term into cost function 
%		3/25/18 - added options for exporting move from plot
%		4/24/18 - speed increase for single step case 
%		7/09/18 - removed multistep optiopns (see git if want)
%		7/12/18 - removed 'do nothing' dominance criteria 
%
% 	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 2/04/18
% 		Stanford University, Biomechatronics Lab 
%******************************************************************************
t_solve = tic;				% to get solve time 	
numKeep = ceil(numKeep); 	% incase not an integer
Debug = false; 

%% --------- Get required input from the actuator that is passed in -----------
lims = actuator.limits; 
lims1 = lims(1); lims2 = lims(2);
n = actuator.numClutches; 
kc = actuator.kc;
kp = actuator.kp;
kf = actuator.kf; 
b = actuator.b;
bb = row(b); 		% so we dont need to always recompute the transpose  
S = actuator.S; 
changeCosts = actuator.changeCosts; 
maxActForce = actuator.GetMaxForce(); % to use  for scaling plots 
ttn = 2^n; % two to the n (we use this variable a lot)
bintable = cell2mat(actuator.stateTable)'; 


%%-------------- Get required info from the force profile --------------------- 
k  = length(forceProfile.time); 		% number of actual time steps 
force = forceProfile.force(:); 	% needs to be a col vector 
dt = forceProfile.time(2) - forceProfile.time(1); 
disps = forceProfile.disp(:); 
T = (k - 1)/(numControl - 1); % Sould be an int 
h = dt * T; % time difference between control steps 
if (abs(T - round(T, 15)) > eps)
	errorStruct.message = ['T is not an integer. Use an h value that is',...
												' an integer multiple of dt'];
	errorStruct.identifier = 'BQBF:invalidStepSize';
	error(errorStruct);
else
 	T = round(T); 
end 



%% ---------------------- Handle Variable Input args --------------------------
numplot = 0; 
if numel(varargin) > 0 
	if varargin{1} == true
		makeplot = true;
		numplot = 4; % set as a default value for numplot 
	else
		makeplot = false; 	
	end 
	if length(varargin) > 1
		numplot = varargin{2};
	end 
else
	makeplot = false;
end 

%% Initial Spring Positions 
if (length(varargin) > 2) && any(varargin{3}) 	
	x0 = varargin{3};
else
 	x0 = zeros(n , 1); 
end  
P_offset = repmat(x0, 1, k); 

%% Initial Clutch States 
if (numel(varargin) > 3) && any(varargin{4}) 
	initClutches = varargin{4};
else
	initClutches = zeros(1, n);
end 

%% Create movie from plots 
makeMovie = false; 
if (length(varargin) > 4)
	if (varargin{5})
		vidname = varargin{6};
		makeMovie = true;
	end
end 


%% ---------------------------------------------------------------------------
[d, D, ~] = getMatrices(forceProfile);


keep = numKeep; 		% from input list 
if numplot > numKeep	
	numplot = keep; % can't plot more than we have 
	warning('Requested plots of more histories than are being tracked');
end 


%% Get heuristic baseline to start (for dominance )
if (keep ~= 1)
	[hsol, ~, heuristicBaseline] = BQBF(actuator, forceProfile, 1,...
													numControl, 0, inf);
	hC = hsol{1}; 
end 

%%----------------------- Initialize Cell Arrays -----------------------------
sols = cell(keep*ttn, 1); 						% storing current solutions
solpos = cell(keep*ttn, 1);
%curpos = cell(max(keep, ttn)*ttn, 1);			% for dominance comparing 
%solendpos = cell(max(keep, ttn)*ttn, 1);  		% to keep track of last pos 
curpos = cell(keep*ttn, 1);			% for dominance comparing 
solendpos = cell(keep*ttn, 1);  		% to keep track of last pos 
prevcosts = zeros(keep*ttn, 1);
%prevcontrol = cell(max(keep, ttn)*ttn, 1);  	% last control step 
%curcontrol = cell(max(keep, ttn)*ttn, 1); 		% needed for dominance 
prevcontrol = cell(keep*ttn, 1);  	% last control step 
curcontrol = cell(keep*ttn, 1); 		% needed for dominance 


bestSols = cell(max(keep, ttn), 1);  		% will have all the sols to start 
bestSolsPos = cell(max(keep, ttn), 1); 		
bestSolsEndPos = cell(max(keep, ttn), 1); 
bestPrevcontrol = cell(max(keep, ttn), 1); 
bestSolsposcosts = zeros(max(keep, ttn), 1);





bCosts = zeros(1, numel(bestSols)); 		% costs for initial best sols 

lsols = numel(sols); 
if lsols > 2^12
	warning('There will be %i solutions to check at each step', lsols);
end 




% Create initial 
C = zeros(n, numControl); % empty control matrix 

PP = zeros(n, T); 		% T+ 1? 
PP(:, 1) = x0; 	% account for initial positions 
% TODO -- add check on num control being more than time points in trajectory 


for i = 1:max(keep, ttn) 
	if i <= ttn
		newcontrol = bintable(:, i); % the clutch combination
		newC = C;                
		newC(:, 1) =  newcontrol; 

		bestSols{i} = newC; 
		bestPrevcontrol{i} = newcontrol; % make a col vec
		
		%% eval cost for sorting	
		for j = 2:T % or T + 1? 
			PP(:, j) = PP(:, j - 1) + (d(j - 1) * newcontrol); 
		end 
		rawPosCost = computeRawPosCost(PP, lims, n);
		newPos = PP(:, T) + (d(T) * newcontrol); 	% start pos for next step

		changeVec = newcontrol - initClutches(:);	% for cost on control
		outForce = (newcontrol' * b) + S * bsxfun(@times, newcontrol, PP); 

		forceError = force(1:T) - outForce(:);
	    
	    % Fill in for 'Best'
	    bestSolsposcosts(i) = rawPosCost; 

	    %bestSolsPos{i} = P; 		% should need to keep the whole matri? 
	    bestSolsEndPos{i} = newPos;

		% Compute the actual cost 
		bCosts(i) = computeCost(forceError, kf, changeVec,...
							 changeCosts, kc, rawPosCost, kp, dt);

	else % fill up with garbage to maintain generality 
		bestSols{i} = inf(n, k); 
		bestSolsposcosts(i) = inf; 
		bestSolsPos{i} = inf*PP; 
		bestSolsEndPos{i} = inf * newPos; 
		bestPrevcontrol{i} = inf * initClutches(:);
		bCosts(i) = inf; 
	end
end 

% dont want to sort with the nans so only sort 1:ttn
[~, idxs] = sort(bCosts(1:ttn), 'ascend'); 
if keep > ttn 
	extra = keep - ttn; 
	idxs = [idxs,  ((ttn+1):keep)]; % need the nans to be last
end 

%% Sort all of the cell arrays according to cost
bestSols = bestSols(idxs);  
bestSolsposcosts = bestSolsposcosts(idxs); 
bestSolsPos = bestSolsPos(idxs);
bestSolsEndPos = bestSolsEndPos(idxs);
bestPrevcontrol = bestPrevcontrol(idxs); 
lastCosts = bCosts(idxs);


initialSols = bestSols; % for debug

if Debug
	fprintf('Step: 1,  Minval: %0.4f\n',  min(lastCosts));
end 

%% Now we actually solve
% we already took the first control step so the next control step is 2 

for ctrl = 2:numControl

	solcosts = zeros(lsols, 1); 		% initialize for storing costs 
	
	for j = 1:lsols 		
		offset = floor((j-1)/ttn); 
	
		startPos = bestSolsEndPos{offset + 1};  	% start pos for this step 
		prevcosts(j) = lastCosts(offset + 1);	% cost to arrive 
		nextSol = bestSols{offset + 1};  		% on the first move
		prevcontrol{j} = bestPrevcontrol{offset + 1};
	
		stp = floor((j-1) - offset*ttn);   
		newcontrol = bintable(:, stp + 1);
		curcontrol{j} = newcontrol;		% Need this for dominance comparisons 


		ctrlOffset = T * (ctrl - 1); 

		if ctrl < numControl % not last control decision
			PP(:, 1) = startPos; 	% account for initial positions 
			for i = 2:T % or T + 1? 
				PP(:, i) = PP(:, i - 1) + (d(ctrlOffset + i - 1) * newcontrol); 
			end 
			% start pos for next step 
			endPos = PP(:, T) + (d(ctrlOffset + T) * newcontrol); 	
		else 
			PP = zeros(n, 1); 	% need to expand it by one column
			PP(:, 1) = startPos; 	% account for initial positions 
			endPos = PP(:, end); 	
		end 

		nextSol(:, ctrl) = newcontrol;
		sols{j} = nextSol; 		% store away the whole matrix 	
		solendpos{j} = endPos;

		if ~isinf(prevcosts(j))	% if prevcost is inf, dont waste comp time 	
			addedRawCost = computeRawPosCost(PP, lims,  n);  

			if ~isinf(addedRawCost)

				% need for applying dominance criteria
				curpos{j} = solendpos{j}; 	
				changesNew = newcontrol - prevcontrol{j}; 
			
				outForce = (newcontrol' * b) + ...
										S * bsxfun(@times, newcontrol, PP); 
				if ctrl < numControl
					idxs = (ctrlOffset + 1):1:(ctrlOffset + T);
				else 
					idxs = k; 
				end 
				forceErrorNew = force(idxs) - outForce(:);
				prevcontrol{j} = newcontrol; % update for next loop 
				newCosts = computeCost(forceErrorNew, kf, changesNew,...
								 changeCosts, kc, addedRawCost, kp, dt);
				solcosts(j) = prevcosts(j) + newCosts;
			else
				solcosts(j) = inf; 
			end 

			if Debug
				fprintf(['Ctrl: %i, Pos: %s, Force: %0.4f,',...
						'Force Error New: %0.4f\n'], ctrl,...
						vecprint(solendpos{j}, 3),...
						 outforce, dt * forceErrorNew^2); 
			end
		else
			% NOTE -- maybe need to put some other things here 
			solcosts(j) = inf;	
		end 	
	end 

	% No point in applying dominance criteria for keep = 1 (heuristic)
	% TODO -- probably remove the ttn^n ctrl - 1 line 
	if (keep > 1) && (keep*ttn^(ctrl - 1) > lsols) && ~isinf(domRatio) 

		%% First Dominance Criteria 
		lists = cell(ttn, 1);
		arrayIdxList = ones(ttn, 1); % to keep track of indexing into arrays 
		for i = 1:ttn
			lists{i} = zeros(1, lsols); 
		end 

		for i = 1:lsols		
			num = binvec2num(curcontrol{i}) + 1; 

			% Need to properly index into it... this is why you used 
			lists{num}(arrayIdxList(num))  = i;
			arrayIdxList(num) =  arrayIdxList(num) + 1; 
		end 

		for i = 1:ttn
			curList = lists{i}(1:(arrayIdxList(i) - 1)); 
			lists{i} = curList; 
			rnks = solcosts(curList); 

			% if not the lowest, get rid of it 
			[curScore, I] = min(rnks); 
			pos = curpos{curList(I)};		
			numKill = 0; 

			for j = 1:length(curList)
				if j ~= I
					otherPos = curpos{curList(j)}; 
					posdiff = sum((pos - otherPos).^2); 
					scoreDiff = solcosts(curList(j)) - curScore; 			
					if (scoreDiff/posdiff) > domRatio 
						solcosts(curList(j)) = inf; 
						numKill = numKill + 1; 
					end 											
				end 

			end 
			% fprintf('CTRL %i, At %i, Num Kill %i out of %i\n', ctrl,...
					%						 i, numKill, length(curList));
		end 
				
		numInf = 0;
		for i = 1:lsols
			if isinf(solcosts(i))
				numInf = numInf + 1;  		% FOR DEBUG 
			end 
		end 
		numLost = lsols - keep - numInf;	% FOR DEBUG 
	end % end if keep > 1 

	[minvals, idxs] = multimin(solcosts, keep); % now we have the best sols

	if Debug
		fprintf('Step: %i,  Minval: %0.4f\n', ctrl, minvals(1));
	end 


	% TODO remove this because it slows down
	if length(idxs) ~= length(unique(idxs))
		fprintf('Repeated Indices\n')
	end

	


	% pick out the best solutions and put them into bestSols 
	bestSols = sols(idxs);
	bestSolsPos = solpos(idxs);
	bestSolsEndPos = solendpos(idxs);
	bestPrevcontrol = prevcontrol(idxs); 
	%bestCurcontrol = curcontrol(idxs); 
	lastCosts = minvals;
	
	 
	%% Add plotting options 
	if makeplot 
		% this is super slow but its for plotting...
		[costs, idxs] = multimin(solcosts, numplot); 

		if ctrl == 2
			fig = figure; hold all 
			t = forceProfile.time; 

			plot(t, forceProfile.force, 'k--', 'DisplayName', 'Target Force');			
			handles = cell(numplot, 1);

			for j = 1:numplot
				P = sols{idxs(j)}*D;
				outforce = sols{idxs(j)}'*b + (S*(sols{idxs(j)}.*P))';
				str = sprintf('Cost: %0.2f', costs(j)); 
				handles{j} = stairs(t, outforce, 'DisplayName', str); 
				set(handles{j}, 'YData',  outforce); 
			end 
			xlabel('Time'); ylabel('Force');
			ylim([0, 1.2*maxActForce]);
			legend show; leg = legend; 
			set(leg, 'Interpreter', 'latex'); 
			drawnow;

			if makeMovie
				dir1 = 'bqbfFrames';
				mkdir(dir1);
				system(['rm ', dir1, '/*']);
			end
		else 
			for j = 1:numplot 
				P = sols{idxs(j)}*D;
				outforce = sols{idxs(j)}'*b + (S*(sols{idxs(j)}.*P))';
				str = sprintf('Cost: %0.2f', costs(j)); 
				set(handles{j}, 'DisplayName', str);
				set(handles{j}, 'YData',  outforce); 				
			end 
			% refreshdata 
			drawnow;
			pause(0.005); 
		end 
	
		if makeMovie
			framestr = sprintf('%s/frame%0.5i', dir1, ctrl);
			print(framestr, '-dpng', '-r300');
		end 
	end % if makeplot 

	
end % end the main while loop 

bestCost = min(minvals); 
solveTime = toc(t_solve); 


%% Now create force profiles to output 
% bestSols = bestSols(1:keep); % TODO decide if should include this line 

forces = cell(keep,  1);

PP = zeros(n , k); 
PP(:, 1) = x0; 
for i = 1:keep
	C = bestSols{i};

	
	outForce = zeros(k, 1);
	for j = 1:k % or T + 1? 
		u = C(:, ceil(j/T));
		if j > 1
			PP(:, j) = PP(:, j - 1) + (d(j - 1) * u); 
		end 
		outForce(j) =  (b' * u) + S * (u .* PP(:, j)); 
	end 
	%outforce = C'*b + (S*(C.*P))'; 
	forces{i} = outForce; 
end 


%% Add variable outputs here 
if (nargout - 2) > 0
	varargout{1} = solveTime; 
end 

if (nargout - 2) > 1
	if makeplot
		varargout{2} = fig; % handle for figure that was plotted on  
	else
		varargout{2} = [];
	end
end 


if makeMovie
	movieTime = 15; % seconds 
	folder2movie(dir1, vidname, 'time', movieTime);
end 


%% Error chhecking for best solutoins to help debugging 
%{
b = actuator.b;
S = actuator.S; 
C_best = bestSols{1}; 
totCost = costFromMatrices(C_best, D, force, b, S, kf, kc, kp, dt, lims);
display(totCost - bestCost); % should be zero
%}
