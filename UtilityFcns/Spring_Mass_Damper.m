


m = 1; 
b = 0.1; 
k = 1; 

A = [0, 1; -k/m, -b/m]; 

fun = @(t, x) A*x; 


y0 = [1; 0]; 
t_0 = 0;
t_f = 10; 
dt = 0.1; 

[y_out, t] = RK4(fun, y0, t_0, t_f, dt); 
[y_out2, t2] = RK4(fun, y0, t_0, t_f, 0.5); 
[y_out3, t3] = RK4(fun, y0, t_0, t_f, 1); 

figure, 
plot(t, y_out(1, :)), hold all
plot(t2, y_out2(1, :)),
plot(t3, y_out3(1, :)),
hold off 
