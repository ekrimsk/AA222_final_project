%**************************************************************************
%	Function:
%		frame2movie.m
%
%	Description:
%		
%
%   Revisions:
%		
%
%	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 3/25/18
% 		Stanford University, Biomechatronics Lab 
%**************************************************************************
function [] = frames2movie(frames, vidname, varargin)

frameRate = 30; % default if not changed
frameCount = 0; 
for i = 1:numel(frames)
	if (~isempty(frames(i).cdata))
    	frameCount = frameCount + 1; 
    end
end 

if nargin > 2
   if strcmpi(varargin{1}, 'framerate') 
   	  frameRate = varargin{2};        
   elseif strcmpi(varargin{2}, 'time')  
   	  frameRate = ceil(frameCount/time);open(v); 
   end
end 
 
v = VideoWriter(vidname); 
v.FrameRate = frameRate; 
fprintf('\nCreating video: %s\n', vidname); 
open(v); 
for i = 1:frameCount 
    writeVideo(v, frames(i)); 
end 
close(v); 
