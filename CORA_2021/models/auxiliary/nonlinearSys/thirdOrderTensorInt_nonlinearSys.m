function [Tf,ind] = thirdOrderTensorInt_nonlinearSys(x,u)



 Tf{1,1} = interval(sparse(4,4),sparse(4,4));



 Tf{1,2} = interval(sparse(4,4),sparse(4,4));



 Tf{1,3} = interval(sparse(4,4),sparse(4,4));



 Tf{1,4} = interval(sparse(4,4),sparse(4,4));



 Tf{2,1} = interval(sparse(4,4),sparse(4,4));



 Tf{2,2} = interval(sparse(4,4),sparse(4,4));



 Tf{2,3} = interval(sparse(4,4),sparse(4,4));



 Tf{2,4} = interval(sparse(4,4),sparse(4,4));


 ind = cell(2,1);
 ind{1} = [];


 ind{2} = [];

