function rawpc = computeRawPosCost(pos, lims, n)
%**************************************************************************
%	Function:
%		computeRawPosCost.m
%
%	Description:
%		Computes the unweighted postion cost from the clutch positions 
%		matrix and the specified displacement limits
%
%	Inputs:
%		pos - position matrix of size (num clutches) x (num time steps)
%		lims - 1 x 2 vector of actuator limits where lims(1) < lims(2)
%	
%	Outputs:
%		rawpc - raw position cost (unweighted)
%
%   Revisions:
%		4/5/18 - sped up computation by reformulating into matrix ops 
%		4/10/18 - added  check for infinite input pos (like in BQBF)
%
%	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 3/21/18
% 		Stanford University, Biomechatronics Lab 
%**************************************************************************

n = size(pos, 1); % TODO remove n from the inputs

%{
rawpc = 0; 
tic
for i = 1:size(pos, 2)
	newPos = pos(:, i);

	if ((max(newPos) > lims(2)) || (min(newPos) < lims(1)))
		rawpc = inf; 
		break; 
	end 
	addedCost =  (sum((lims(1) * lims(2))./ ...
						(n * (lims(1) - newPos) .* (lims(2) - newPos))) - 1); 
	rawpc = rawpc + addedCost; 
	%rawpc = rawpc + ( (lims(1) * lims(2)/n) * sum(bsxfun(@rdivide, 1, bsxfun(@times, bsxfun(@minus, lims(1), newPos), bsxfun(@minus, lims(2), newPos)) ))   -  1);

end 
disp(rawpc)
toc
%}

%% Alternate appraoch 
%tic

if isinf(pos(1,1))
	rawpc = inf; 
	return;
end  

%% Quadratic Cost Fun 
A = max(max(pos));
if A > lims(2)
	rawpc = inf; 
	return;
end

B = min(min(pos));
if B < lims(1)
	rawpc = inf;
	return;
end 

rawpc = sum(sum(pos.^2));


%% Non quadratic cost function 
%{
A = lims(1) - pos;
B = lims(2) - pos; 
C =  1./(A.*B); 



if any(any(C > 0))
	rawpc = inf;
	return;
else
	if (size(C, 2) == 1)
		rawpc = ((lims(1) * lims(2)/n) * sum(C)) - 1;
	else	
		D = ((lims(1) * lims(2)/n) * sum(C, 1)) - 1;
		rawpc = sum(D, 2);
	end
end 
%disp(RAWPC)
%toc
%}



if rawpc < 0
	errorStruct.message = 'Negative postion cost returned';
	errorStruct.identifier = 'computeRawPosCost:badMathNegativeCost';
	error(errorStruct);
end 
