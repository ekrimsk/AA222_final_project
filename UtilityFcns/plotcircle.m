function [circ] = plotcircle(x,  y, r)

	bx = x - r;
	by = y - r;
	circ = rectangle('Position', [bx, by, 2*r, 2*r], 'Curvature', 1); 


	%% Default values
	% perimeter  = black
	% face = white 
	% visible = off (can be turned on with 'set')
		set(circ, 'facecolor', [1, 1, 1], 'visible', 'off');
end 