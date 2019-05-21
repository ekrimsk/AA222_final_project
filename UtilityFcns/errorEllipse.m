function [Xplot, Yplot] = errorEllipse(Xdata, Ydata)
%*************************************************************************
%	Function:
%		errorEllipse.m
%
%	Description:
%		Generates points for plotting ellipses that encompass about
%		95% percent of the input data (2 standard deviations).
%		Base code taken from mathworks forums 
%
%	Inputs:
%		Xdata - vector of x data 
%		Ydata - vector of y data 
%
%	Outputs:
%		Xplot - x data for ellipse to be plotted 
%	    Yplot - y data for ellipse to be plotted 
%
%	Author: 
%		Erez Krimsky, ekrimsky@stanford.edu, 4/3/18 
% 		Stanford University, Biomechatronics Lab 
%*************************************************************************
if size(Xdata, 2) > size(Xdata, 1)
	Xdata = Xdata';
end 

if size(Ydata, 2) > size(Ydata, 1)
	Ydata = Ydata';
end 





X = [Xdata, Ydata];
mu = mean(X); 
X0 = X - mu; 

STD = 2;                     %# 2 standard deviations
conf = 2*normcdf(STD)-1;     %# covers around 95% of population
sc = chi2inv(conf,2); 
curcov = sc * cov(X0); 


[V,  D] = eig( curcov );    
[D, idxs] = sort(diag(D), 'descend');
D = diag(D);
V = V(:, idxs);
t = linspace(0,2*pi,100);

e = [cos(t) ; sin(t)];        %# unit circle
VV = V*sqrt(D);               %# scale eigenvectors
e = (VV*e) + mu'; %#' project circle back to orig space

Xplot = e(1,:);
Yplot = e(2,:);