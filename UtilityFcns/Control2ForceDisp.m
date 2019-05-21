function [force, displacements] = Control2ForceDisp(C, b, D, Q, S)

force = C'*b + (S*(C.*(C*D)))'; % for non-constant force 
displacements = C*D; 