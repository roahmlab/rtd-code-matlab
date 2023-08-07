function [A,B]=jacobian_tankSystem(x,u)

A=[1 - 299226832345163/(18014398509481984*x(1)^(1/2)),0,0,0,0,-1/200;...
(3*2^(1/2)*109^(1/2))/(40*x(1)^(1/2)),1 - (3*2^(1/2)*109^(1/2))/(40*x(2)^(1/2)),0,0,0,0;...
0,299226832345163/(18014398509481984*x(2)^(1/2)),1 - 299226832345163/(18014398509481984*x(3)^(1/2)),0,0,0;...
0,0,299226832345163/(18014398509481984*x(3)^(1/2)),1 - 299226832345163/(18014398509481984*x(4)^(1/2)),0,0;...
0,0,0,299226832345163/(18014398509481984*x(4)^(1/2)),1 - 299226832345163/(18014398509481984*x(5)^(1/2)),0;...
0,0,0,0,299226832345163/(18014398509481984*x(5)^(1/2)),1 - 299226832345163/(18014398509481984*x(6)^(1/2))];

B=[1/2;...
0;...
0;...
0;...
0;...
0];
