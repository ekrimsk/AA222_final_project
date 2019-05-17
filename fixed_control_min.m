%*******
%
%
%
%
%
%
%
%****************************





% Compute X in the calling function with X = U*D --> spring positions at every step 

% X is n x (h + 1)






% function [b, k, f_mids_out] = fixed_control_min(f_des, U, X, b_init, k_init)
%X_mid = (X(:, 1:end-1) - X(:, 2:end))/2;	% diff may be faster here 
function [b, k, f_mids_out] = fixed_control_min(f_des, U, X_mid, b_init, k_init)


% enforce f and k to be col vecs 
b_init = b_init(:); 
k_init = k_init(:); 






% Comment this out but good sanity check for now 
f_init = (b_init' * U) + k_init' * (U .* X_mid); 


%display(f_des)
%display(f_init)
obj_init = norm(f_des - f_init);

display(obj_init); 


% number of springs n 

[n, h] = size(U); 


% number of steps h 

cvx_begin quiet 
	variable b(n, 1)	% optimized baseline force 
	variable k(n, 1)	% optimized spring stiffness 

	variable f(1, h);	% midpoint force out -- maybe make an expression ???? 

	%minimize norm(f_des - f, 2)	% minimize the 2-norm
	minimize (norm(f_des - f, 2))	% minimize the 2-norm

	subject to
		f == (b' * U) + (k' * (U .* X_mid)); 

		% TODO -- 

		% some feasibility constraints on b and k 

		% could add some ranking constraints b and k -- monotically increasing 

		% TODO -- other possible constraint is norm on how far we are from initial solution

cvx_end 

% display(f)

display(cvx_status)
display(cvx_optval)



f_mids_out = (b' * U) + (k' * (U .* X_mid)); 


f_out = (b' * U) + (k' * (U .* X_mid)); 