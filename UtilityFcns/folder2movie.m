%**************************************************************************
%	Function:
%		folder2movie.m
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
function [] = folder2movie(folder, vidname, varargin)

files = dir([folder, '/*.png']);
frameCount = numel(files);




frameRate = 30; % default if not changed
if nargin > 2
    if strcmpi(varargin{1}, 'framerate') 
   	   frameRate = varargin{2};        
    elseif strcmpi(varargin{1}, 'time')  
       time = varargin{2} ;
   	   frameRate = ceil(frameCount/time);
   end
end 
 
v = VideoWriter(vidname); 
v.FrameRate = frameRate; 
fprintf('\nCreating video: %s, with framerate %i\n', vidname, frameRate);
open(v); 

for i = 1:frameCount
    writeVideo(v, imread(fullfile(files(i).folder, files(i).name))); 
end 
close(v); 
