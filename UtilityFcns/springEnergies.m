function energies = springEnergies(L, L0, A)
%**************************************************************************
% Function:
%	springEnergies
%	
%	Description:
%	
%
%	Inputs:
%	
%		
%	Outputs:
%	
%	
%	
%
%
%	Revisions:
%		7/10/18 - Removed springLims from input list 
%
% 	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 6/26/18
% 		Stanford University, Biomechatronics Lab 
%**************************************************************************
C1 = ClutchSpringConstants.C1; 
C2 = ClutchSpringConstants.C2;

E1 = C2 * (L0.^3)./(L.^2) - (C2 *L0); 
E2 = 2 * C1 * (L0.^2)./L - (2*C1*L0); 
E3 =  ((C1 * L.^2)./L0) - (C1 * L0);
E4 = (2 * C2 * L) - (2 * C2 * L0); 

energies = A.*(E1 + E2 + E3 + E4); 
