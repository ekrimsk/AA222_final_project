 classdef ClutchSpringConstants
    properties( Constant = true )
         
    	%% ------------- Spring Material Constants ---------------------
		k1 = 269e3;
		k2 = 102e3;
		C1 = (243/2)*1e3; 		% Pa
		C2 = (223/2)*1e3; 		% Pa 
		springRho = 1.1e3;  	% spring density kg/m^3

		%% ---------------- Clutch Constants -------------------
		cf = 7800; 				% N/m^2 (clutch force factor )
		clutchRho = 3;          % kg/m^2  
		E_film = 4.9 * 10^9;	% film modulus 
		t_film = 25e-6; 		% film thickness  


        %% Clutch width 
        cw = 0.11; 
         
    end
 end