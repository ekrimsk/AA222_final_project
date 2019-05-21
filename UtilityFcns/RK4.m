function [y_out, t] = RK4(fun, y0, t_0, t_f, dt)
%**************************************************************************
% 	Function:
%		RK4.m
%
%	Description:
%		Uses an RK4 integration scheme to perform forward integration
%
%	Inputs:
%		fun - function handle where y_dot_n = fun(t_n, y_n)
%		y0 - initial condition
%		t_0 - initial time
%		t_f - final time 
%		dt - step size 
%		
%	Outputs:
%		y_out - output of the forward integration
%		t - corresponding time vector for output 
%
%	Notes:
%		If dt does not fit cleanly in the interval, a warning will be 
%		issued and t (out) will end at a value in the range of 
%		(t_f - dt) and t_f
%
%	Revisions:
%
% 	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 7/03/18
% 		Stanford University, Biomechatronics Lab 
%**************************************************************************
t = t_0:dt:t_f; 	%NOTE maybe issue warning when time step doesnt line up 


if ( (t_f - t(end)) > eps)
	warning('RK4: timestep does not fit cleanly into time bounds'); 
	% TODO -- add interpolation for this case 
end 



pts = length(t);
n = length(y0); 

if iscolumn(y0)
	y_out = zeros(n, pts); 
	y_out(:, 1) = y0; 
	for i = 1:(pts - 1)
		yi = y_out(:, i);
		ti = t(i); 
		
		k1 = dt * fun(ti, yi); 
		k2 = dt * fun(ti + dt/2, yi + k1/2); 
		k3 = dt * fun(ti + dt/2, yi + k2/2); 
		k4 = dt * fun(ti + dt, yi + k3); 
		y_out(:, i + 1) = yi + (k1 + 2*k2 + 2*k3 + k4)/6; 
	end 
elseif isrow(y0)
	y_out = zeros(pts, n); 
	y_out(1, :) = y0; 

	for i = 1:(pts - 1)
		yi = y_out(i, :);
		ti = t(i); 
		
		k1 = dt * fun(ti, yi); 
		k2 = dt * fun(ti + dt/2, yi + k1/2); 
		k3 = dt * fun(ti + dt/2, yi + k2/2); 
		k4 = dt * fun(ti + dt, yi + k3); 
		y_out(i + 1, :) = yi + (k1 + 2*k2 + 2*k3 + k4)/6; 
	end 
else
	errorStruct.message = 'Invalid initial condition size';
	errorStruct.identifier = 'RK4:invalidInitialCond';
	error(errorStruct);	
end 
