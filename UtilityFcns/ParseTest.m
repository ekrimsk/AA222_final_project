function [] = ParseTest(varargin) %numClutches, limits, base, kf, kc, kp, varargin
	defaultKf = 1;
	defaultKc = 1; 
	defaultKp = 1; 
	defaultBase = 2; 
	defualtFr = 0;
	defaultVel = false; 
	defaultType = 'single';
	expectedTypes = {'single', 'double'};

	p = inputParser; 

	%% Required Inputs 
	addRequired(p, 'clutches');
	addRequired(p, 'limits'); % add checking on limits 

	%% Optional Inputs (need defualt values )
	addParameter(p, 'base', defaultBase);
	addParameter(p, 'kc', defaultKc);
	addParameter(p, 'kp', defaultKp);
	addParameter(p, 'usevel', defaultVel);
	addParameter(p, 'type', defaultType, @(x) any(validatestring(x,expectedShapes)));


	%% parse, then access with p.Results.thing
	parse(p, varargin{:});
	disp(p.Results);