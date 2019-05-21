clc; clearvars;
import java.util.LinkedList;
LL = LinkedList(); 
C = {}; 
M = []; 


numPoints = 50000;
rnd = rand(1, numPoints); 

disp('Cell grow:')
tic
for i = 1:numPoints
    C{end + 1} = rnd(i); 
end 
toc 

disp('Cell preallocate:')
CC = cell(numPoints, 1); 
tic
for i = 1:numPoints
    CC{i} = rnd(i); 
end 
toc

disp('Java Linked List:')
tic
for i = 1:numPoints
    LL.push(rnd(i)); 
end 
toc 


disp('Custom Linked List Add:')
ML = MLinkedList();
tic
for i = 1:numPoints
    ML.add(rnd(i)); 
end 
toc 



disp('Custom Linked List Remove:')
tic
for i = 1:numPoints
    ML.remove(); 
end 
toc 


disp('Matrix copy and extend:')
tic
for i = 1:numPoints
    M = [M, rnd(i)];
end 
toc


CCfront = CC; 
CCback = CC;


disp('Cell clear front'); tic
for i = 1:numPoints
    CCfront(1) = []; 
end 
toc

disp('Cell clear back'); tic
for i = 1:numPoints
    CCback(end) = []; 
end 
toc


