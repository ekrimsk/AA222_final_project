classdef Actuator < matlab.mixin.Copyable 
%******************************************************************************
% Class: Actuator
%
% Overview: Object oriented definition of Actuator class for actuator
%	consisting of a parralel arangement of springs and clutches 
%
%
% Constructor Arguments:
%		See comments above the constructor below
%
% Properties:
%	numClutches - number of clutches in the actuator
%	limits - displacements from neutral position. for example
%				lims = [-1, 1] would be used to allow for 1 unit (m) of 
% 				displacement in either direction
%	base - actuator forces are base^integer (2 is standard). Not used
%			if forces are specified directly in constructor
%	kf - force error weighting parameter, see computeCost.m
%	ke - position error weighting parameter 
%	kc - clutch state change error weighting parameter, see computeCost.m
%	kp - position error weighting parameter (distance from neutral 
%			postion), see computeCost.m
%   xee - displacement of end effector. Starts at 0 units 
%	vel - velocity of the end effector 
%

%
%	stateTable - cell array where each entry is an array of size 
%					numClutches x 1. The state table gives all possible							
%					combinations of engaged/disengaged clutches. For an
%					actuator with three clutches the entries will be
%			        as follows...
%							[0 0 0]
%							[0 0 1]
%							[0 1 0]
%							[0 1 1]
%							[1 0 0]... etc.
%
%
%	Revisions:
%		4/2/18 - added call to computeChangeCosts 
%		6/22/18 - changed plot, adds physical dimensions to constructor
%		7/09/18 - Added positino control MPC 
%		7/10/18 - removed steps and history 
%
% 	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 1/12/18
% 		Stanford University, Biomechatronics Lab 
%******************************************************************************
properties (GetAccess = public, SetAccess = private)

	% TODO add starting clutch positions as an input 

	%%------------------------ Fixed Parameters ---------------------------
	numClutches
	limits
	

	% for storing force ranges for control inputs
	minSpringForces
	maxSpringForces

	minForces
	midForces
	maxForces 

	%% Physical Dimensions for clutch plates and springs 
	w_vec
	t_vec
	L0_vec
	L_cs_vec	% input clutch lengths
	L_co_vec	% output clutch lengths
	L_cf_vec	% frame clutche lengths 
	L_cf_top	% distance from top of frame clutch 
	L_spring_offset
	ps_vec	
	output_init 
	output_max
	output_min
	%L_max_vec
	%L_ol_vec

	b % column vector of nominal forces 
	S % stifnes matrix (row vector)
	stateTable

	%% Cost Function params 
	kf
	ke
	kc
	kp
	fr

	base
	changeCosts
	changeCostTable

	%% ------------------ Control Loop Parameters -------------------------
	freq 				% Hz


	%% --------------------- Variable Parameters --------------------------
	xee			% end effector position
	vel 		% end effector velocity 
	cpos 		% current clutch positions 
	cstatus 	% binvec of current clutch status 
	forceHistory 
	controlHistory
	
	%% for plotting 
	mainTransform
	length
	extensionLength
	anchorPoint
end 


methods (Access = public)
	%**********************************************************************
	% Constructor for the class. 
	%
	% Required Inputs: 
	%	numClutches - number of clutches
	%	forceType - either base (base raised to a power) or vector 
	%						(vector speciftying the nominal force)
	%	forceParams - if base type, a scalar number, if vector type,
	%				a nun clutches x 1 vector specifying nominal forces
	%
	% Optional Inputs ('name' - value pairs):
	% 	limits - displacement limits for end effector
	%	base - actuator forces are base^integer (2 is standard)
	%	kf - force error weighting parameter, see computeCost.m
	%	ke - position error weighting term
	%	kc - clutch state change error weighting parameter, 
	%						see computeCost.m
	%	kp - position error weighting parameter (distance from neutral 
	%		postion), see computeCost.m
	%
	% Returns:
	% 	actuator object
	%********************************************************************
	%function obj = Actuator(numClutches, limits, base, kf, kc, kp, varargin) % can expand this later
	function obj = Actuator(varargin) % we will parse the inputs directly 
		
		% set the properties 
		if nargin > 0 % allows for zero argument constructor for allocation

			%% Default Values (if not included in input list)
			forceTypes = {'base', 'vector'};
			defaultBase = nan;

			defaultKf = 1;
			defaultKe = 1;
			defaultKc = 1; 
			defaultKp = 1; 
			defaultClutches = 1;
			defaultLims = [-1 , 1]; 
			defaultS = 0;  
			defaultFr = 0;
			defaultType = 'single'; % not implemented now, add later 
			expectedTypes = {'single', 'double'};

			defaultFreq = 10;		% Hz  

			p = inputParser; 

			%% Validation Fcns
			validLimits = @(x) (numel(x) == 2) && (x(1) <= 0) && (x(2) >= 0) && (x(1) < x(2) );				
			validClutches =  @(x) isnumeric(x) && isscalar(x);

			%% Required Positional inputs
			addRequired(p, 'clutches', validClutches);
			addRequired(p, 'forceType',  @(x) any(validatestring(x,forceTypes)));
			addRequired(p, 'forceParams', @(x) isnumeric(x)); % -- corresponding with forceType


			%% Optional Inputs (need default values )
			addParameter(p, 'limits', defaultLims, validLimits); % add checking on limits 
			
			% addParameter() -- functionality for different starting position

			%% Use FR or S 
			addParameter(p, 'fr', defaultFr, @(x) isnumeric(x));
			addParameter(p, 'S', defaultS, @(x) isnumeric(x));
			addParameter(p, 'type', defaultType, @(x) any(validatestring(x,expectedTypes)));

			%% Cost Function parameters 
			addParameter(p, 'kf', defaultKf, @(x) isnumeric(x)); 
			addParameter(p, 'ke', defaultKe, @(x) isnumeric(x)); 
			addParameter(p, 'kc', defaultKc, @(x) isnumeric(x));
			addParameter(p, 'kp', defaultKp, @(x) isnumeric(x));
			

			%% Control Loop Frequency
			addParameter(p, 'frequency', defaultFreq, @(x) isnumeric(x));

			%% Physical dimensions 
			addParameter(p, 'w', 0,  @(x) isnumeric(x));
			addParameter(p, 't', 0,  @(x) isnumeric(x));
			addParameter(p, 'L0', 0,  @(x) isnumeric(x));
			addParameter(p, 'L_cs', 0,  @(x) isnumeric(x));
			addParameter(p, 'L_co', 0,  @(x) isnumeric(x));
			addParameter(p, 'L_cf', 0,  @(x) isnumeric(x))
			addParameter(p, 'L_spring_offset', 0, @(x) isnumeric(x)); 
			%addParameter(p, 'L_max', 0,  @(x) isnumeric(x));
			%addParameter(p, 'L_ol', 0,  @(x) isnumeric(x));	
			addParameter(p, 'ps', 0,  @(x) isnumeric(x));
			addParameter(p, 'output_init', 0,  @(x) isnumeric(x));
			addParameter(p, 'output_max', 0,  @(x) isnumeric(x));
			addParameter(p, 'output_min', 0,  @(x) isnumeric(x));

			%% parse, then access with p.Results.thing
			parse(p, varargin{:});
			obj.numClutches = p.Results.clutches;
			obj.limits = p.Results.limits;
			obj.fr = p.Results.fr;
			obj.base = defaultBase; 
			
			% variables for cost function use
			obj.kf = p.Results.kf;
			obj.ke = p.Results.ke;
			obj.kc = p.Results.kc; 
			obj.kp = p.Results.kp; 

			% control frequency 
			obj.freq = p.Results.frequency; 

			% dimensions
			obj.w_vec = p.Results.w;
			obj.t_vec = p.Results.t;
			obj.L0_vec = p.Results.L0;
			obj.ps_vec	= p.Results.ps;

			obj.L_cs_vec = p.Results.L_cs;	% input clutch lengths
			obj.L_co_vec = p.Results.L_co;	% output clutch lengths
			obj.L_cf_vec = p.Results.L_cf; 
			L_ps = obj.ps_vec .* obj.L0_vec; 
			obj.L_cf_top = max(obj.L_cf_vec) * ones(obj.numClutches, 1);
			obj.L_spring_offset = p.Results.L_spring_offset; 


			obj.output_init = p.Results.output_init;
			obj.output_max = p.Results.output_max;
			obj.output_min = p.Results.output_min; 

			%% Always initialized to the following...
			obj.xee = 0; 	% end effector to start 
			obj.vel = 0; 	% initial velocity zero (may want to add input?)

			if strcmp(p.Results.forceType, 'base')
				tmp = (obj.numClutches - 1):-1:0;
				obj.b = (p.Results.forceParams.^tmp)';
				obj.base = p.Results.forceParams; 
				if p.Results.S == 0 % meaning it wasnt set by input params 
					obj.S = (obj.fr * obj.b')/obj.limits(2);
				else 
					obj.S = reshape(p.Results.S, 1, obj.numClutches); 
				end 
			else
				if numel(p.Results.forceParams) ~= obj.numClutches
					errorStruct.message = 'Incorrect number of force inputs';
					errorStruct.identifier = 'Actuator:invalidForceInputs';
					error(errorStruct);
				else
					obj.b = reshape(p.Results.forceParams, obj.numClutches, 1);
					obj.S = reshape(p.Results.S, 1, obj.numClutches); 
				end 
			end 

			obj.cpos  = zeros(obj.numClutches, 1);		% current clutch positions 				
			obj.cstatus = zeros(obj.numClutches, 1); 	% binvec of current clutch status 
			
			%% Generate the state table  
			numStates = 2^obj.numClutches; 
			states = cell(numStates, 1); 
			for i = 1:numStates
				states{i} = num2binvec(i - 1, obj.numClutches);
			end
			obj.stateTable = states; 


			obj.changeCosts = zeros(1, obj.numClutches);
			for i = 1:obj.numClutches
				obj.changeCosts(i) = obj.GetClutchForce(i, obj.limits(2)); 
				% NOTE -- we may restructure change costs
			end 

			n = obj.numClutches;
			obj.changeCostTable = zeros(n, n);
			for i = 1:n
				vec1 = num2binvec(i, n); 
				for j = 1:n
					vecdiff = abs(vec1 - num2binvec(j, n)); 
					obj.changeCostTable(i, j) = (vecdiff*obj.changeCosts');
				end 
			end 

			% populate min and max forces 
			obj.minSpringForces = zeros(n, 1);
			obj.maxSpringForces = zeros(n, 1); 

			for i = 1:n
				obj.minSpringForces(i) = obj.b(i) + ...
										(obj.S(i) * obj.limits(1));
				obj.maxSpringForces(i) = obj.b(i) + ...
										(obj.S(i) * obj.limits(2));
			end 

			obj.minForces = zeros(numStates, 1); 
			obj.midForces = zeros(numStates, 1); 
			obj.maxForces = zeros(numStates, 1); 

			for i = 1:numStates
				control = states{i}; 
				for j = 1:n 
					if (control(j) == 1)
						obj.minForces(i) = obj.minForces(i) + ...
												obj.minSpringForces(j);
						obj.midForces(i) = obj.midForces(i) + obj.b(j); 
						obj.maxForces(i) = obj.maxForces(i) + ...
												obj.maxSpringForces(j);
					end 
				end 
			end 
		end % end if nargin > 0 
	end %end constructor 

	


	%**********************************************************************
	% Method: PositionMPC
	% 
	% Inputs: 
	%	obj - the class object
	%	traj - trajectory structure 
	%	dynamics - struct of function handles 
	%	tEnd - time where springs should return to initial positions
	%	h - timestep size for RK4 
	% 	P - terminal penalty weight matrix on spring positions
	%	varargin{1} - an incumbent solution for control 
	%	varargin{2} - number of control steps to plan for. The default 
	%					is the whole trajectory, the minimum is one 
	%	varargin{3} - if this input is present, this  value is the max 
	%					planning time. It overwritesd the value set 
	%					by the actuator frequency. 
	%	varargin{4} - use inneficies in reporting y_out (true or false). 
	%					if true, cost check will differ from branch and 
	%					bound cost
	%
	% Returns:
	%	X - output spring positions rows 1:n, 
	%         end effector position row n + 1  and velocity row n + 2 
	%	U - control n x number of control steps (depends on length 
	%						of planning horizon)
	%	optVal - cost from the optimizer 
	%
	%	Revisions:
	%		7/26/18 - adding inneficiency into branch and bound 
	%
	%   Added: 7/5/18 	
	%**********************************************************************
	function [X, U, optVal] = PositionMPC(obj, traj, dynamics,...
								 tEnd, h, P, varargin) 



		% TODO -- addin error checks on output 

		% get oopy of params from object
		n = obj.numClutches; 
		x0 =  obj.cpos; 			
		u_minus = obj.cstatus; 		
		xee = obj.xee; 				
		xee_dot = obj.vel; 			
		tSolveMax = 1/obj.freq; 

		% consider add some initial interp on the traj to ensure alignment 
		
		dt = traj.dt;
		T = h/dt;   % issue if not an int 

		if ( abs(T - round(T, 15)) > eps);  
			error('T is not an integer');
		else
			T = round(T);
		end 

		cycleIdx = round((traj.tEnd - traj.time(1))/dt) + 1;  % FOR DEBUG 
		t_init = traj.time(1);
		cycleLen = (traj.cycleTime/dt) + 1; 		% TODO + 1 or no? 



		if (numel(varargin) > 1) && (~isempty(varargin{2}))	
			% truncate the trajectory 
			num = varargin{2}; 	% number of control to use 
			t_final = t_init + (h * num); 
			% truncate the trajectory 
			traj.disp = traj.disp(1:(num*T + 1)); 
			traj.time = traj.time(1:(num*T + 1)); 
		else
			t_final = traj.time(end);
		end     

	    t = t_init:h:t_final; 				% should fit in evenly
	    pts = length(t) - 1; 

	    N = length(traj.time);

	    if (numel(varargin) > 2)			% truncate the trajectory 
			if (~isempty(varargin{3}))
				tSolveMax = varargin{3}; 
			end
		end     


		L_ps_vec = obj.L0_vec .* obj.ps_vec; 
		useInef = false; 			% default 
		if (numel(varargin) > 3)
			if (varargin{4} == true) 
				useInef = true;	
			end 
		end 


		if (numel(varargin) > 0) && (~isempty(varargin{1}))
			% use incumbent 
			%disp('use incumbent')
			[U, optVal, data] = BranchBoundPositionControl(obj, traj,...
								dynamics, pts + 1, ...
						 		'x0', x0,...
								'xee0', xee,...
							    'xee_dot0', xee_dot, ...
								'Uminus', u_minus,...
								'P', P,...
								'Inefficiency', useInef, ... 
								'incumbent', varargin{1},...
								'MaxTime', tSolveMax,...
								'Verbosity', 1); 
		else
			% dont use incumbent 
			%disp('no inc')
			[U, optVal, data] = BranchBoundPositionControl(obj, traj,...
						  		dynamics, pts + 1, ...
								'x0', x0,...
								'xee0', xee,...
								'xee_dot0', xee_dot, ...
								'Uminus', u_minus,...
								'P', P, ...
								'Inefficiency', useInef,...
								'MaxTime', tSolveMax,...
								'Verbosity', 1); 
		end 




		u_prev = u_minus;
		X = zeros(n + 2, N); 				% augmented states 
		X(1:n, 1) = x0;						% spring pos 
		X(n + 1, 1) = xee;					% end effector pos
		X(n + 2, 1) = xee_dot;				% velocity

		for i = 1:pts
			t_0 = t(i);						% init time for this segment
			t_f = t(i + 1); 				% final time for this segment

			startIdx = 1 + (i - 1)*T; 
			endIdx = startIdx + T;

			x = X(1:n, startIdx);
			xee = X(n + 1, startIdx);
			xee_dot = X(n + 2, startIdx);
			u = U(:, i); 


			if useInef
				u_diff = u - u_prev; 
				x = handoffShift(x, u_diff, obj.b(:), obj.S(:), obj.w_vec, ...
					L_ps_vec + obj.L_spring_offset, obj.L_cf_top, obj.output_init, xee);
			end 

			y0 = [x; xee; xee_dot]; 
				
			[y_out, t_out] = RK4(actSystem(dynamics, u), y0, t_0, t_f, dt);
			X(:, startIdx:endIdx) = y_out; 	
			u_prev = u; 				
		end 

		% first, just cost from displacements 
		xee_ref = traj.disp; 
		xee_out = X(n + 1, :); 
		costCheck = (obj.ke * dt) * sum((xee_ref(:) - xee_out(:)).^2); 

		R = obj.kc * diag(obj.changeCosts); 	% control penalty weights
		Q = obj.kp * dt * eye(n);				% state penality weights


		U_diff =  U - [u_minus, U(:, 1:end-1)]; 

		Qhalf = sqrtm(Q);
		Rhalf = sqrtm(R); 
		% USEFUL FOR DEBUG 
		ub = positionControlCost(xee_ref, X, U_diff, dt, obj.ke ,...
													 Qhalf, Rhalf, P, cycleIdx, cycleLen); 
		
		fprintf('Cost check  (function call): %0.5f\n\n', ub); 


		% if we want to actually update here 
		%xStep = X(:, T + 1);
		%uStep = U(:, 1); 

	end 	



	%**********************************************************************
	% Method: forceMPC
	% 
	% Inputs: 
	%	obj - the class object
	%
	% Returns:
	% 	
	% 
	% 


	%**********************************************************************
	function [X, U, optVal] = ForceMPC(obj, forceProfile, ...
								 tEnd, h, P, varargin) 


		% 


		% TODO -- addin error checks on output 

		% get oopy of params from object
		n = obj.numClutches; 
		x0 =  obj.cpos; 			
		u_minus = obj.cstatus; 		
		xee = obj.xee; 				
		xee_dot = obj.vel; 			
		tSolveMax = 1/obj.freq; 


	end 


	% TODO -- add a bunch of setters 

	%**********************************************************************
	% Method: setFreq
	% 
	% Inputs: 
	%	obj - the class object
	%	newFreq - frequency for the control loop 
	%
	% Returns:
	% 	none	
	%**********************************************************************	
	function setFreq(obj, newFreq)
		obj.freq = newFreq; 
	end 

	%**********************************************************************
	% Method: setFreq
	% 
	% Inputs: 
	%	obj - the class object
	%	newKe - new ke weight 
	%
	% Returns:
	% 	none
	%**********************************************************************
	function setKe(obj, newKe)
		obj.ke = newKe; 
	end 

	function setKc(obj, newKc)
		obj.kc = newKc; 
	end 

	function setKp(obj, newKp)
		obj.kp = newKp; 
	end 

		function setKf(obj, newKf)
		obj.kf = newKf; 
	end 



	%**********************************************************************
	% Method: plotForceRanges
	%
	% Shows range of forces than can correspond to each control decision
	%
	% Inputs: 
	%	obj - the class object
	%
	% Returns:
	% 	figure plot was created on 
	%**********************************************************************
	function fig = plotForceRanges(obj)

		x = 0:1:(2^obj.numClutches - 1); 
		pos = obj.maxForces - obj.midForces;
		neg = obj.midForces - obj.minForces;
		mid = obj.midForces;

		fig = figure; hold on 
		errorbar(x, mid, neg, pos, 'k.'); 
		xlabel('Control');
		ylabel('Force'); 
		hold off 
	end


	%*********************************************************************
	% Method: GetPositions
	% Compute maximum possible force for current configuration
	% Inputs: 
	%	obj - the class object
	%
	% Returns:
	% 	The positions of the individual clutches. Output in a vector 
	%**********************************************************************		
	function [clutchLocs] = GetPositions(obj)
		 clutchLocs = obj.cpos;
	end


	%**********************************************************************
	% Method: GetClutchForce
	% Inputs: 
	%	obj - the class object 
	%	clutchNum - the clutch of interest (referenced by number in 
	%					clutch set)
	% 	varargin - if left blank, the force is computed for the current
	% 				location of the clutch. If included, the force is
	%				computed for clutch at the new location.	
	%
	% Returns:
	% 	force for clutch of interest
	%**********************************************************************
	function [force] = GetClutchForce(obj, clutchNum, varargin)
		if ~isempty(varargin) % if there is a second input
			clutchLoc = varargin{1};
		else 
			clutchLoc = obj.cpos(clutchNum); 
		end
		force = obj.b(clutchNum) +...
						(clutchLoc*obj.S(clutchNum));
	end 



	%**********************************************************************
	% Method: GetForce 
	% Inputs: 
	%	obj - the class object
	%
	% Returns:
	% 	force being supplied in current configuration of the actuator,
	%	which includes the force contribution from every clutch 
	% 	(unlike the GetClutchForce method)
	%**********************************************************************
	function [suppliedForce] = GetForce(obj)			
		suppliedForce = obj.cstatus*obj.b + ...
						 (obj.S*(obj.cstatus'.*obj.cpos)); 	
	end % end GetForce


	%**********************************************************************
	% Method: GetWorstCaseForce 
	% Inputs: 
	%	obj - the class object
	%
	% Returns:
	% 	The maximum force the actuator can produce when it is in the 
	%	weakest possible configuration with all its clutches bottomed
	%	out in the most relaxed position
	%**********************************************************************
	function [suppliedForce] = GetWorstCaseForce(obj)
		suppliedForce = 0;
		for i = 1:obj.numClutches
			suppliedForce = suppliedForce +...
								obj.GetClutchForce(i, obj.limits(1));
		end
	end % end GetWorstCaseForce

	%**********************************************************************
	% Method: GetMaxForce 
	% Inputs: 
	%	obj - the class object
	%
	% Returns:
	% 	The maximum force the actuator can produce when it is in the 
	%	strongest possible configuration with all its clutches at 
	%	their extension limits
	%*********************************************************************
	function [suppliedForce] = GetMaxForce(obj)
		suppliedForce = 0;
		for i = 1:obj.numClutches
			suppliedForce = suppliedForce +...
								obj.GetClutchForce(i, obj.limits(2));
		end
	end % end GetWorstCaseForce



	%**********************************************************************
	% Method: GetEnergy
	% Inputs: 
	%	obj - the class object
	%
	% Returns:
	% 	Energy stored in the set of clutches. There is assumed to be 
	%	zero energy stored at the neutral position
	%**********************************************************************
	function [energy] = GetEnergy(obj)
		energy = 0;
		for i = 1:obj.numClutches
			baseEnergy = obj.b(i) .* obj.cpos(i);
			extraEnergy = 0.5*obj.S(i)*...
						sign(obj.cpos(i))*(obj.cpos(i))^2;
			energy = energy + baseEnergy + extraEnergy; 
		end
	end % end GetEnergy



	%**********************************************************************
	% Method: plot
	% Inputs: 
	%	obj - the class object
	%	varargin{1} - figure
	%	varagin{2} - anchor point  
	%	vaargin{3} - orientation (0 deg is straight up)
	%
	% Returns:
	% 	none
	%
	%	Revisions:
	%		Change to take in force profile and timestep 
	%**********************************************************************
	function [mainTransform] = plot(obj, varargin)

		
		% pass in 
		if nargin > 1

			set(0, 'Current', varargin{1}); 
			if isempty(varargin{1}.CurrentAxes)
				ax = axes; hold on  
				%axis equal
				%set(ax, 'Xlim', xlims); set(ax, 'Ylim', ylims); 		% FIX 
				set(ax, 'XTickLabel', ''); 			
				set(ax, 'XTick', ''); 
			else
				ax = gca; 
			end
		end



		if nargin == 1
			% note - may ne
			ax = axes; hold on  
			%axis equal
			%set(ax, 'Xlim', xlims); set(ax, 'Ylim', ylims); 		% FIX 
			set(ax, 'XTickLabel', ''); 			
			set(ax, 'XTick', ''); 
		end 


		% Can tweak these defs to make the plots prettier 
		red = [1, 0, 0]; 
		green = [0, 1, 0];
		blue = [0, 0, 1];
		gray = [0.5, 0.5, 0.5]; 
		
		n = obj.numClutches; 

		w_vec = obj.w_vec; 
		L0_vec = obj.L0_vec; 
		L_cs_vec = obj.L_cs_vec;
		L_co_vec = obj.L_co_vec; 
		L_cf_vec = obj.L_cf_vec; 
		L_cf_top = obj.L_cf_top; 
		ps_vec = obj.ps_vec;
		L_spring_offset = obj.L_spring_offset; 

		tc = 0.0125;		% clutch plate plot thickness 


		xSpringCenters = 0:(6*tc):((n - 1)* 6 * tc); 
		xSpringCenters = xSpringCenters - mean(xSpringCenters); 


		mainTransform = hgtransform('Parent', ax);
	
		%% ------------ Create the enclosure ---------------------------
		% need to know the minpoint reachable --
		actEnclosure = hggroup;
		set(actEnclosure, 'tag', 'enclosure'); 
		set(actEnclosure, 'Parent', mainTransform); 
		tt =  tc; 

		minPoint = min([obj.output_min - obj.L_co_vec; 0]);
		maxPoint = obj.output_max; 

		leftEnd = xSpringCenters(1) - 5 * tc; 
		rightEnd = xSpringCenters(end) + 5 * tc; 
		encWidth = rightEnd - leftEnd; 
		encHeight = maxPoint - minPoint + (5 * tc); 

		encOuterPos = [leftEnd, minPoint - 2*tc,...
							 					encWidth, encHeight]; 

		enc1 = rectangle('Position', encOuterPos,...
						 'Parent', actEnclosure); 

		enc1a = rectangle('Position', [(leftEnd + rightEnd)/2 - encWidth/6,...
						 minPoint  - encWidth/4, 2*encWidth/6, encWidth/4],...
						 'Curvature', [0.75, 0.5],...
						 'Parent', actEnclosure); 

		set(enc1, 'FaceColor', [0.5, 0.5, 0.5],...
					'EdgeColor', [1, 1, 1]); % TODO -- add a tag 

		set(enc1a, 'FaceColor', [0.5, 0.5, 0.5],...
					'EdgeColor', [0.5, 0.5, 0.5]); % TODO -- add a tag 



		enc2 = rectangle('Position', [encOuterPos(1) + tt,...
									 encOuterPos(2) + tt, ...
									encWidth - 2*tt, encHeight - 2*tt]); 
		set(enc2, 'Parent', actEnclosure);
		set(enc2, 'FaceColor', [1, 1, 1],...
						'EdgeColor', [1, 1, 1]); % TODO -- add a tag 


		% add a circle for the anchor point
		rad = 0.005;  
		anchorPoint = [(leftEnd + rightEnd)/2, minPoint - encWidth/8];
		obj.anchorPoint = anchorPoint;
		anchorPos = [anchorPoint(1) - rad, anchorPoint(2) - rad,...
										   	2*rad, 2*rad];

		anchor = rectangle('Position', anchorPos, ...
							'Curvature', [1, 1], ...
							'Parent', actEnclosure, ...
							 'tag', 'anchor',...
							 'FaceColor', [1, 1, 1]); 

		uistack(actEnclosure, 'bottom'); 

		
		%%------------------------ Output Clutches -----------------------------
		outputClutchTransform = hgtransform('Parent', mainTransform,...
													'tag', 'output_transform');
		actOutput = hggroup; 	% group for the output clutches 
		set(actOutput, 'Parent', outputClutchTransform);
		set(actOutput, 'tag', 'output');
		outputClutchTransform.Matrix(2, 4) = obj.output_init + obj.xee;

		% left endpoint position of each output clutch
		outputXpos = xSpringCenters + (1.5 * tc); 

		% create the connection bar (create it at zero build everything below)
		connection = rectangle('Position', [outputXpos(1), 0,...
				outputXpos(end) - outputXpos(1) + tc, tc], 'Parent', actOutput);
		set(connection, 'tag', 'connection', 'FaceColor', [0, 0, 0],...
														 'Parent', actOutput); 

		% add an extension to the connection 
		extensionLength = obj.output_max - obj.output_min + (4 * tc); 
		obj.extensionLength = extensionLength; 
		extensionPos = [(anchorPos(1) + 0.5 * anchorPos(3)) - tc/2, 0, ...
														  tc, extensionLength];
		extension = rectangle('Position', extensionPos, ...
								'Parent', actOutput, 'tag', 'extension',...
											'FaceColor', [0, 0, 0]); 

		% now add all the output clutches 
		for i = 1:n
			pos = [outputXpos(i), -L_co_vec(i), tc, L_co_vec(i)];				% NOTE - may want to add an offset 
			outputClutch = rectangle('Position', pos, 'Parent', actOutput);
			tag = ['output_', num2str(i)]; 
			set(outputClutch, 'FaceColor', green, 'EdgeColor', 'none', ...
													'tag', tag); 
		end 	


		%% --------------------------- Springs ----------------------------
		R0 = tc * ones(n, 1); %w_vec/2; 
		NE = ceil(1.5 * L0_vec./R0); 
		A0 = L0_vec; 

		springLen = (ps_vec .* L0_vec) + obj.cpos; % incorporate clutch positions 

		for i = 1:n

			[xSpring, ySpring] = spring_points(xSpringCenters(i), L_spring_offset(i),...
						  xSpringCenters(i), springLen(i) + L_spring_offset(i), NE(i), A0(i), R0(i)); 

			springTransform = hgtransform('Parent', mainTransform,...
								'tag', ['spring_transform_', num2str(i)]);

			spring = plot(xSpring, ySpring, 'k', 'Parent', springTransform); 
			set(spring, 'tag', ['spring_', num2str(i)]);
		end 	

		%% ------------------------ Input Clutches -----------------------------
		for i = 1:n 
			inputClutch = hggroup; 
			inputClutchTransform = hgtransform('Parent', mainTransform,...
								'tag', ['input_transform_', num2str(i)]);
			w = 3 *tc; % w_vec(i);
			L_cs = L_cs_vec(i); 
			rpos1 = [xSpringCenters(i) - w/2, 0, w, L_cs];
			rpos2 = [xSpringCenters(i) - w/2 + (w/3), tc,...
													 w - (2 * w/3), L_cs - tc];


			inputClutchTransform.Matrix(2, 4) = springLen(i) + L_spring_offset(i); 

			r1 = rectangle('Position', rpos1, 'Parent', inputClutch); 
			r2 = rectangle('Position', rpos2, 'Parent', inputClutch);
			set(r1, 'FaceColor', gray, 'EdgeColor', 'none'); 
			set(r2, 'FaceColor', [1, 1, 1], 'EdgeColor', 'none'); 
			set(inputClutch, 'Parent', inputClutchTransform); 
			set(inputClutch, 'tag', ['input_', num2str(i)]); 
		end 

		%% ------------------------- Frame Clutches ----------------------------
		actFrame = hggroup; 
		set(actFrame, 'Parent', mainTransform, 'tag', 'actFrame'); 

		for i = 1:n
			w = 3 * tc; 
			L_cf = L_cf_vec(i);
			fc_pos = [xSpringCenters(i) - w/2 - tc, L_cf_top(i) - L_cf,...
														 tc, L_cf];  
			frameClutch = rectangle('Position', fc_pos, 'Parent', actFrame);	% TODO - che idright parent  
			set(frameClutch , 'FaceColor', red, ...
											'EdgeColor', 'none', 'tag',...
											['frame_clutch_', num2str(i)]); 
			uistack(actFrame, 'top'); 
		end			


		if nargin > 2 % this arg is where to put the anchor point 
			desiredAnchor = varargin{2}; 
			xOffset = desiredAnchor(1) - (anchorPos(1) + anchorPos(3)/2);
			yOffset = desiredAnchor(2) - (anchorPos(2) + anchorPos(4)/2); 
			mainTransform.Matrix(1, 4) = xOffset;
			mainTransform.Matrix(2, 4) = yOffset; 	
		end 

		% orientation to put actuator at (default vertical ) -- degress from vertical ccw 
		if nargin > 3 
			rotDeg = varargin{3};
			Rz = makehgtform('zrotate', deg2rad(rotDeg)); 
			mainTransform.Matrix = Rz * mainTransform.Matrix;
		end 


		% TODO make extension length an class propery? 

		obj.mainTransform = mainTransform; 
		obj.length = extensionLength + outputClutchTransform.Matrix(2, 4)...
											 - obj.anchorPoint(2); 

		%% Get total frame width (for plot, not physical )
	end % end plot 

	% TODO -- method for updating plot? (maybe change underlying dimensinos then call an update function)


	%**********************************************************************
	% Method: updateClutches 
	% Inputs: 
	%	u - new clutch states 
	%
	% Returns:
	% 	none
	%**********************************************************************
	function  updateClutches(obj, u)

		% TODO add error checking 
		obj.cstatus = u(:);

		new_green = [0. 0.7, 0.1]; 
		new_red = [0.7, 0, 0.1]; 
		% update the plot colors as well 
		% only if plot obj exists though 

		if ~isempty(obj.mainTransform)

			for i = 1:obj.numClutches
				clutch = findall(obj.mainTransform, 'tag', ['input_', num2str(i)]);  

				if (u(i) == 1)
					set(clutch.Children(2), 'FaceColor', new_green); 
				else
					set(clutch.Children(2), 'FaceColor', new_red); 
				end 
			end 

		end 

	end 



	%**********************************************************************
	% Method: updateAll
	% Inputs: 
	%	y - [x; xee; xee_dot]
	%
	% Returns:
	% 	none
	%**********************************************************************
	function  updateAll(obj, y)
		n = obj.numClutches; 
		y = y(:);
		obj.cpos = y(1:n);
		obj.xee = y(n + 1);
		obj.vel = y(n + 2); 
	end 




	%**********************************************************************
	% Method: 
	%
	% Inputs: 
	%
	%
	% Returns:
	% 	none
	%
	%**********************************************************************
	function  updateSprings(obj, springPos)

		% TODO add error checking 
		obj.cpos = springPos(:);
	end 



	%**********************************************************************
	% Method: 
	%
	% Inputs: 
	%
	%
	% Returns:
	% 	none
	%
	%**********************************************************************
	function updateOutput(obj, newOutput)
		% TODO - add error checking 
		obj.xee = newOutput; 
	end 


	%**********************************************************************
	% Method: updatePlot 
	%
	% Inputs: 
	%	none 
	%
	% Returns:
	% 	none
	%**********************************************************************
	function updatePlot(obj)
		%% update input cluthes
		% define the neutral position for each spring/input clutch 
		% so we define displacement wrt neutral position 
		% for bottom of each -- define it off ps 
		neutralPos = (obj.ps_vec .* obj.L0_vec) + obj.L_spring_offset; 
		newPos = neutralPos + obj.cpos; 

		%% -------------------------- Springs ----------------------------------
		for i = 1:obj.numClutches
			springTransform = findall(obj.mainTransform, 'tag',...
								 ['spring_transform_', num2str(i)]);

		  	%springTransform.Matrix(2, 2) = newPos(i)/neutralPos(i);  % scaling might not work well if not anchored at zero\
		  	springTransform.Matrix(2, 2) = (newPos(i) - obj.L_spring_offset(i))/(neutralPos(i) - obj.L_spring_offset(i));	% miight work?, TODO if so restructure to make more readbale
		end 

		%% ------------------ Input (Spring) Clutches --------------------------
		for i = 1:obj.numClutches
			inputTransform = findall(obj.mainTransform, 'tag',...
								 ['input_transform_', num2str(i)]);
		  	inputTransform.Matrix(2, 4) = newPos(i);  
		end 

		%% ------------------------ Output --------------------------------
		outputTransform = findall(obj.mainTransform, 'tag',...
												 'output_transform'); 
		outputTransform.Matrix(2, 4) = obj.output_init + obj.xee;

		%% update the length
		obj.length	= obj.extensionLength + ...
					outputTransform.Matrix(2, 4) - obj.anchorPoint(2); 
		drawnow; 
	end 	


	
	%**********************************************************************
	% Method: ActCopy
	%  Get the clutch parameters 
	%
	% Inputs: 
	%	obj - the class object
	%
	% Returns:
	% 	Copied actuator (does not copy history)
	%**********************************************************************
	function actuatorCopy = ActCopy(obj)
		actuatorCopy = obj.copy();
	end
end % end public methods 

%==============================================================================
end % end CLASSDEF
