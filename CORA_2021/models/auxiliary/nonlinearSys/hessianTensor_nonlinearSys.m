function Hf=hessianTensor_nonlinearSys(x,u)



 Hf{1} = sparse(4,4);

Hf{1}(3,2) = 1;
Hf{1}(2,3) = 1;
Hf{1}(4,3) = 1;
Hf{1}(3,4) = 1;


 Hf{2} = sparse(4,4);

Hf{2}(2,1) = -1;
Hf{2}(1,2) = -1;
Hf{2}(4,4) = 2;
