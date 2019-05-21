function [pcost] = PartialCost(outForce, force, controlMatrix, kc, Q)

numchange = nnz(controlMatrix*Q);
%disp(numchange);
pcost = (force - outForce)'*(force - outForce) + kc*numchange;