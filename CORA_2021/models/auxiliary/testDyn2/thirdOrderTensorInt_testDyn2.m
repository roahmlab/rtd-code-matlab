function [Tf,ind] = thirdOrderTensorInt_testDyn2(x,u)



 out = funOptimize(x,u);



 Tf{1,1} = interval(sparse(6,6),sparse(6,6));



 Tf{1,2} = interval(sparse(6,6),sparse(6,6));



 Tf{1,3} = interval(sparse(6,6),sparse(6,6));



 Tf{1,4} = interval(sparse(6,6),sparse(6,6));



 Tf{1,5} = interval(sparse(6,6),sparse(6,6));



 Tf{1,6} = interval(sparse(6,6),sparse(6,6));



 Tf{2,1} = interval(sparse(6,6),sparse(6,6));



 Tf{2,2} = interval(sparse(6,6),sparse(6,6));



 Tf{2,3} = interval(sparse(6,6),sparse(6,6));



 Tf{2,4} = interval(sparse(6,6),sparse(6,6));



 Tf{2,5} = interval(sparse(6,6),sparse(6,6));



 Tf{2,6} = interval(sparse(6,6),sparse(6,6));



 Tf{3,1} = interval(sparse(6,6),sparse(6,6));

Tf{3,1}(1,1) = out(1);
Tf{3,1}(2,1) = out(2);
Tf{3,1}(1,2) = out(3);
Tf{3,1}(2,2) = out(4);


 Tf{3,2} = interval(sparse(6,6),sparse(6,6));

Tf{3,2}(1,1) = out(5);
Tf{3,2}(2,1) = out(6);
Tf{3,2}(1,2) = out(7);
Tf{3,2}(2,2) = out(8);


 Tf{3,3} = interval(sparse(6,6),sparse(6,6));



 Tf{3,4} = interval(sparse(6,6),sparse(6,6));



 Tf{3,5} = interval(sparse(6,6),sparse(6,6));



 Tf{3,6} = interval(sparse(6,6),sparse(6,6));



 Tf{4,1} = interval(sparse(6,6),sparse(6,6));

Tf{4,1}(1,1) = out(9);
Tf{4,1}(2,1) = out(10);
Tf{4,1}(1,2) = out(11);
Tf{4,1}(2,2) = out(12);


 Tf{4,2} = interval(sparse(6,6),sparse(6,6));

Tf{4,2}(1,1) = out(13);
Tf{4,2}(2,1) = out(14);
Tf{4,2}(1,2) = out(15);
Tf{4,2}(2,2) = out(16);


 Tf{4,3} = interval(sparse(6,6),sparse(6,6));



 Tf{4,4} = interval(sparse(6,6),sparse(6,6));



 Tf{4,5} = interval(sparse(6,6),sparse(6,6));



 Tf{4,6} = interval(sparse(6,6),sparse(6,6));



 Tf{5,1} = interval(sparse(6,6),sparse(6,6));



 Tf{5,2} = interval(sparse(6,6),sparse(6,6));



 Tf{5,3} = interval(sparse(6,6),sparse(6,6));



 Tf{5,4} = interval(sparse(6,6),sparse(6,6));



 Tf{5,5} = interval(sparse(6,6),sparse(6,6));



 Tf{5,6} = interval(sparse(6,6),sparse(6,6));


 ind = cell(5,1);
 ind{1} = [];


 ind{2} = [];


 ind{3} = [1;2];


 ind{4} = [1;2];


 ind{5} = [];

function out = funOptimize(in1,uL1R)
%FUNOPTIMIZE
%    OUT = FUNOPTIMIZE(IN1,UL1R)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    04-Aug-2021 12:38:43

xL1R = in1(1,:);
xL2R = in1(2,:);
t2 = xL1R.*2.0;
t3 = xL2R.^2;
t4 = xL2R.^3;
t5 = xL1R.*8.0;
t7 = xL1R+4.2164e+7;
t14 = xL1R.*1.43496e+18;
t6 = -t3;
t8 = t7.^2;
t9 = t2+8.4328e+7;
t10 = t5+3.37312e+8;
t22 = t14+6.050365344e+25;
t11 = t9.^2;
t12 = t9.^3;
t13 = t6+t8;
t15 = 1.0./t13.^(5.0./2.0);
t16 = 1.0./t13.^(7.0./2.0);
t17 = 1.0./t13.^(9.0./2.0);
t18 = t15.*4.30488e+18;
t19 = t15.*1.291464e+19;
t21 = t3.*t16.*2.15244e+19;
t24 = t9.*t16.*xL2R.*2.15244e+19;
t25 = t9.*t16.*xL2R.*3.22866e+19;
t26 = t11.*t16.*5.3811e+18;
t28 = t4.*t9.*t17.*7.53354e+19;
t29 = t3.*t11.*t17.*3.76677e+19;
t31 = t16.*t22.*xL2R.*1.5e+1;
t32 = t9.*t16.*t22.*(1.5e+1./2.0);
t33 = t3.*t9.*t17.*t22.*(1.05e+2./2.0);
t34 = t11.*t17.*t22.*xL2R.*(1.05e+2./4.0);
t20 = -t18;
t23 = -t21;
t27 = -t26;
t30 = -t29;
t35 = -t34;
t36 = t25+t28;
t37 = t18+t21+t27+t30;
t38 = t24+t31+t35;
t39 = t20+t23+t32+t33;
mt1 = [t19-t32-t11.*t16.*1.61433e+19-t10.*t16.*t22.*(1.5e+1./4.0)+t12.*t17.*t22.*(1.05e+2./8.0),t38,t38,t39,t38,t39,t39,t4.*t17.*t22.*-1.05e+2-t16.*t22.*xL2R.*4.5e+1,t9.*t16.*xL2R.*-1.07622e+19-t10.*t16.*xL2R.*5.3811e+18+t12.*t17.*xL2R.*1.883385e+19,t37,t37,t36,t37,t36,t36];
mt2 = [-t19-t3.*t16.*1.291464e+20-t3.^2.*t17.*1.506708e+20];
out = reshape([mt1,mt2],16,1);