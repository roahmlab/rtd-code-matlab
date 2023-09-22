function [A,B]=jacobian_nonlinearSys(x,u)

A=[0,u(1);...
- x(2) - 1,1 - x(1)];

B=[u(2) + x(2),u(1);...
0,2*u(2)];

