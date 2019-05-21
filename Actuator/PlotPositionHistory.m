%**************************************************************************
%	Function:
%		PlotPositionHistory.m
%
%	Description:
%		Plots position history of clutches from a matrix of positions 
%
%	Inputs:
%		positions - (num time steps) x (num clutches) matrix of positions.
%		forceProfile - force profile structure with forces, disps, time
%		str - string to put in legend on plot 
%		lims - 1 x 2 vector for y-axis limits on plot 
%
%	Outputs	
%		fighandle - handle to figure plot was made on 
%
%   Revisions:
%		
%
%	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 3/21/18
% 		Stanford University, Biomechatronics Lab 
%**************************************************************************

function [fighandle] = PlotPositionHistory(positions, forceProfile, str, lims)


if size(positions, 2) > size(positions, 1)
	positions = positions'; % in case the wrong size was used as input 
end 


fighandle = figure; hold all
numPositions = size(positions, 1);
for i = 1:size(positions, 2);
	fullstr = sprintf('%s, Clutch: %i', str, i);
	plot(forceProfile.time(1:numPositions), positions(:, i),...
												 'DisplayName', fullstr);
end
ylim(lims);
xlabel('Time'); ylabel('Position');
legend show; 
