function [A,B]=jacobian_fiveDimSysEq(x,u)

A=[-1,-4,0,0,0;...
4,-1,0,0,0;...
0,0,-3,1,0;...
0,0,-1,-3,0;...
0,0,0,0,-2];

B=[1,0,0,0,0;...
0,1,0,0,0;...
0,0,1,0,0;...
0,0,0,1,0;...
0,0,0,0,1];

