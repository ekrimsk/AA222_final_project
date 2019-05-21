function [success, varargout] = RunActuator(actuator, forceProfile, varargin) 
%*****************************************************************************
%	Function:
%		RunActuator.m
%
%
%	Description:
%		Takes an actuator and runs it through the control steps 
%		corresponding the given force profile using a heuristic controller
%
%	Inputs:
%		actuator - input actuator object. The object will be changed by the
%					function call 
%
%		forceProfile - force profile structure
%		varargin{1} - a control matrix to override the heuristic controller
%						if desired 
%
%
%	Outputs:
%		success - true (if no error encountered), false if control fails 
%		varargout{1} - total cost for control
%		varargout{2} - matrix giving location history for clutches 
%
%
% 	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 1/24/18
% 		Stanford University, Biomechatronics Lab 
%*****************************************************************************
success = true; % change to false if fails 
totalCost = 0; 
steps = length(forceProfile.time); 
varargout = cell(nargout - 1, 1); 

if length(varargin)
	controlInput = varargin{1}; % instead of using heursitic cost func
end 

if nargout > 2
	locationHistory = zeros(steps, length(actuator.GetPositions()));
end 

for i = 1:steps
	try
		if length(varargin)
			[stepCost, ~] = actuator.StepActuator(forceProfile, i, ...
								controlInput(i)); %
		else
			[stepCost, ~] = actuator.StepActuator(forceProfile, i);
		end 
		totalCost = totalCost + stepCost; 
		if nargout > 2
			locationHistory(i, :) = actuator.GetPositions;
		end 
	catch ME
		success = false; % did not manage to get through all steps 
		if strcmp(ME.identifier, 'StepActuator:limitExceeded')
			%warning(ME.message); 
			break; % stop surrounding loop
		else
			rethrow(ME); 
		end
	end
end 


if nargout > 1
	varargout{1} = totalCost; 
end 

if nargout > 2
	varargout{2} = locationHistory;
end 