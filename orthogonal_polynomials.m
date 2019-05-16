
close all; clear all; clc; 
% Script for playing around with generating random f(t) and v(t) such that they are orthogonal 


h = 1000; % pts to gen 
t = linspace(-1, 1, h);


% make a sample v 


% v = 10 * legendreP(2, t) + 3 * legendreP(4, t); 

v_coeffs = rand(3, 1);
v =  v_coeffs(1) * legendreP(2, t) + v_coeffs(2) * (legendreP(5, t)  - legendreP(9, t)); %+  v_coeffs(3) * legendreP(6, t); 


v = legendreP(2, t)  + legendreP(5, t)  - legendreP(9, t);
% v = legendreP(1, t);
%v = v - v(1);		% not required but could be good to start at zero velocity 





f =  legendreP(1, t) +  legendreP(3, t) - legendreP(4, t) + legendreP(7, t);
f = f - f(1) + 10; 
% f = f + 1

%f= ones(1, h);
% f = legendreP(3, t)


% Add arbitrary sign flips???? 


figure, hold on 
plot(t, v, 'r');
plot(t, f, 'b');

hold off 



net_work = trapz(t, f.*v);
display(net_work)

