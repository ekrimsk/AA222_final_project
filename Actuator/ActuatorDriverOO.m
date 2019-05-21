%*************************************************************************
% 	File:
%		ActuatorDriverOO.m
%
% 	Description:
% 		Driver code for object oriented version of actuator in response to 
% 		force profile. Can make video of actuator with "makeMovie = true"
%
%	Revisions:
%		1/31/18 - import force profile, pass dircectly to actuator obj
%		2/15/18 - changes to account for changes in actuator class defs
%
% 	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 1/12/18
% 		Stanford University, Biomechatronics Lab 
%**************************************************************************

close all; clearvars; clc;
addpath(genpath(pwd));
addpath('ForceProfileCodes') 
load('Workspaces/forGA1_nc_4__mf_15__dl_4.mat'); % load in a force profile 

%% Parameter for force profile 

cycles = 2;
forceProfile = cycleProfile(forceProfile, cycles) 
maxForce = 3; 
forceProfile.force = maxForce * forceProfile.force/max(forceProfile.force);
forceProfile.force = 0.9*forceProfile.force; %% need max force to be less than what can be delivered at neutral position


%% Parameters for output 
makePlot = true; 
makeMovie = true;  % change to save movie
frameDelay = 0.01;



%% Actuator Input Parameters -- make sure they match imported force profile 
clutches = 2; 
limits = [-6, 6]; % for actuator (will want to pass this to gen profiles as well)
base = 2;
kf = 1;
kc = 0.000001;
kp = 0;


%% Create an instance of the actuator 
actuator = Actuator(clutches, 'base', base, 'limits', limits, 'fr', 0.1, 'kf', kf,...
									'kc', kc, 'kp' ,kp);



EndCyclePositions = zeros(clutches, cycles);
maxForceError = 0; 
maxCycleErrors = zeros(cycles, 1);

costThresh = 1.5; 

if makePlot % open a figure to plot on 
	fig = figure('Position',[200,100,800,600]);; % the  handle to plot the object we will update 
	F(length(forceProfile)) = struct('cdata',[],'colormap',[]);
end

for i = 1:length(forceProfile.time)
	cycleNum = ceil(i/numSteps);
    if mod(i - 1, numSteps) == 0
    	maxCycleError = 0;    	
        fprintf('\nCycle: %i\n', cycleNum);
        %fprintf('Max force error: %0.1f\n', maxForceError); 
    end

    if (mod(i, numSteps) == 0)
    	maxCycleErrors(i) = maxCycleError;
    	fprintf('Max Cycle Error: %0.2f\n', maxCycleError);

    	actPos = actuator.GetPositions();
    	EndCyclePositions(:, cycleNum) = actPos;
    	fprintf('Positions: [ %s ]\n', num2str(actPos'));			 
	end

    %% get values from force profile 

	

	%% Propogate step on the main actuator
	try
		[stepCost, ~] = actuator.StepActuator(forceProfile, i);
		%fprintf('\nMin cost: %0.2f', stepCost);
	catch ME
		warning('Simulation ended:')
		if strcmp(ME.identifier, 'StepActuator:limitExceeded')
			warning(ME.message);

		else
			rethrow(ME);
			break; % stop surrounding loop
		end
	end


	fsupplied = actuator.GetForce();	
	currForce = forceProfile.force(i);
    
    if (abs(fsupplied - currForce) > maxCycleError)
    	maxCycleError = abs(fsupplied - currForce);
    	if (abs(fsupplied - currForce) > maxForceError)
        	maxForceError = abs(fsupplied - currForce);
    	end
    end 
    
    if makePlot
		actuator.plot(forceProfile, i, fig); 
		if makeMovie
			drawnow
    		F(i) = getframe(gcf);  		
    	end 
    	pause(frameDelay);
	end
end 


fhist = actuator.GetForceHistory; 
newfig = figure; hold all
plot(forceProfile.time, forceProfile.force, 'k--', 'DisplayName', 'Target force')
stairs(forceProfile.time(1:length(fhist)), fhist, 'DisplayName', 'Heuristic');
xlabel('Time'); ylabel('Force');
legend show;  hold off 
print('sim_force_out', '-dpng', '-r300');

if makeMovie
	vidname = 'simvid.avi';
	fprintf('\nCreating movie: %s\n', vidname);
	% v = VideoWriter(vidname,'Uncompressed AVI');      
	v = VideoWriter(vidname);   
	v.FrameRate = 8;               
	open(v);
	for i = 1:length(forceProfile.force)
		if ~isempty(F(i).cdata)
			writeVideo(v, F(i))
		else
			break; % quit writing frames
		end
	end
	close(v);
end
