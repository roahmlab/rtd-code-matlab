function [A,B]=jacobian_tank6paramEq(x,u,p)

A{1}=[-(9*2^(1/2)*109^(1/2))/(4000*x(1)^(1/2)),0,0,0,0,-1/100;...
(9*2^(1/2)*109^(1/2))/(4000*x(1)^(1/2)),-(9*2^(1/2)*109^(1/2))/(4000*x(2)^(1/2)),0,0,0,0;...
0,(9*2^(1/2)*109^(1/2))/(4000*x(2)^(1/2)),-(9*2^(1/2)*109^(1/2))/(4000*x(3)^(1/2)),0,0,0;...
0,0,(9*2^(1/2)*109^(1/2))/(4000*x(3)^(1/2)),-(9*2^(1/2)*109^(1/2))/(4000*x(4)^(1/2)),0,0;...
0,0,0,(9*2^(1/2)*109^(1/2))/(4000*x(4)^(1/2)),-(9*2^(1/2)*109^(1/2))/(4000*x(5)^(1/2)),0;...
0,0,0,0,(9*2^(1/2)*109^(1/2))/(4000*x(5)^(1/2)),-(9*2^(1/2)*109^(1/2))/(4000*x(6)^(1/2))];

A{2}=[0,0,0,0,0,0;...
0,0,0,0,0,0;...
0,0,0,0,0,0;...
0,0,0,0,0,0;...
0,0,0,0,0,0;...
0,0,0,0,0,0];

B{1}=[1;...
0;...
0;...
0;...
0;...
0];

B{2}=[0;...
0;...
0;...
0;...
0;...
0];

