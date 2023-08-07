function Hf=hessianTensorInt_highorderBicycleDynamics(x,u)



 Hf{1} = interval(sparse(20,20),sparse(20,20));

Hf{1}(4,1) = 871661994975820683240720631749/(4724220171757606344889204736*x(4)^2);
Hf{1}(4,3) = -(20266198323167232*((188833848407094893720457330196646994532815626287*u(2))/5708990770823839524233143877797980545530986496 + (188833848407094893720457330196646994532815626287*x(8))/5708990770823839524233143877797980545530986496 - 11303683735006311231131354555373/713623846352979940529142984724747568191373312))/(29035976812752505*x(4)^3);
Hf{1}(8,3) = 292654978791248591199807276445083/(25353012004564588029934064107520*x(4)^2);
Hf{1}(20,3) = 292654978791248591199807276445083/(25353012004564588029934064107520*x(4)^2);
Hf{1}(1,4) = 871661994975820683240720631749/(4724220171757606344889204736*x(4)^2);
Hf{1}(3,4) = -(20266198323167232*((188833848407094893720457330196646994532815626287*u(2))/5708990770823839524233143877797980545530986496 + (188833848407094893720457330196646994532815626287*x(8))/5708990770823839524233143877797980545530986496 - 11303683735006311231131354555373/713623846352979940529142984724747568191373312))/(29035976812752505*x(4)^3);
Hf{1}(4,4) = (60798594969501696*x(3)*((188833848407094893720457330196646994532815626287*u(2))/5708990770823839524233143877797980545530986496 + (188833848407094893720457330196646994532815626287*x(8))/5708990770823839524233143877797980545530986496 - 11303683735006311231131354555373/713623846352979940529142984724747568191373312))/(29035976812752505*x(4)^4) - (20266198323167232*(u(1) + x(7))*((32517219865694287911089697382787*u(2))/2535301200456458802993406410752 + (32517219865694287911089697382787*x(8))/2535301200456458802993406410752 - 23108598393799996055320794791637/79228162514264337593543950336))/(29035976812752505*x(4)^3) - (871661994975820683240720631749*x(1))/(2362110085878803172444602368*x(4)^3);
Hf{1}(7,4) = (10133099161583616*((32517219865694287911089697382787*u(2))/2535301200456458802993406410752 + (32517219865694287911089697382787*x(8))/2535301200456458802993406410752 - 23108598393799996055320794791637/79228162514264337593543950336))/(29035976812752505*x(4)^2);
Hf{1}(8,4) = (292654978791248591199807276445083*(u(1) + x(7)))/(65383207177125271813266593546240*x(4)^2) - (292654978791248591199807276445083*x(3))/(12676506002282294014967032053760*x(4)^3);
Hf{1}(19,4) = (10133099161583616*((32517219865694287911089697382787*u(2))/2535301200456458802993406410752 + (32517219865694287911089697382787*x(8))/2535301200456458802993406410752 - 23108598393799996055320794791637/79228162514264337593543950336))/(29035976812752505*x(4)^2);
Hf{1}(20,4) = (292654978791248591199807276445083*(u(1) + x(7)))/(65383207177125271813266593546240*x(4)^2) - (292654978791248591199807276445083*x(3))/(12676506002282294014967032053760*x(4)^3);
Hf{1}(4,7) = (10133099161583616*((32517219865694287911089697382787*u(2))/2535301200456458802993406410752 + (32517219865694287911089697382787*x(8))/2535301200456458802993406410752 - 23108598393799996055320794791637/79228162514264337593543950336))/(29035976812752505*x(4)^2);
Hf{1}(8,7) = -292654978791248591199807276445083/(65383207177125271813266593546240*x(4));
Hf{1}(20,7) = -292654978791248591199807276445083/(65383207177125271813266593546240*x(4));
Hf{1}(3,8) = 292654978791248591199807276445083/(25353012004564588029934064107520*x(4)^2);
Hf{1}(4,8) = (292654978791248591199807276445083*(u(1) + x(7)))/(65383207177125271813266593546240*x(4)^2) - (292654978791248591199807276445083*x(3))/(12676506002282294014967032053760*x(4)^3);
Hf{1}(7,8) = -292654978791248591199807276445083/(65383207177125271813266593546240*x(4));
Hf{1}(19,8) = -292654978791248591199807276445083/(65383207177125271813266593546240*x(4));
Hf{1}(4,19) = (10133099161583616*((32517219865694287911089697382787*u(2))/2535301200456458802993406410752 + (32517219865694287911089697382787*x(8))/2535301200456458802993406410752 - 23108598393799996055320794791637/79228162514264337593543950336))/(29035976812752505*x(4)^2);
Hf{1}(8,19) = -292654978791248591199807276445083/(65383207177125271813266593546240*x(4));
Hf{1}(20,19) = -292654978791248591199807276445083/(65383207177125271813266593546240*x(4));
Hf{1}(3,20) = 292654978791248591199807276445083/(25353012004564588029934064107520*x(4)^2);
Hf{1}(4,20) = (292654978791248591199807276445083*(u(1) + x(7)))/(65383207177125271813266593546240*x(4)^2) - (292654978791248591199807276445083*x(3))/(12676506002282294014967032053760*x(4)^3);
Hf{1}(7,20) = -292654978791248591199807276445083/(65383207177125271813266593546240*x(4));
Hf{1}(19,20) = -292654978791248591199807276445083/(65383207177125271813266593546240*x(4));


 Hf{2} = interval(sparse(20,20),sparse(20,20));



 Hf{3} = interval(sparse(20,20),sparse(20,20));

Hf{3}(8,1) = 20107228577383576526557010503895406560826078403/2854495385411919762116571938898990272765493248;
Hf{3}(20,1) = 20107228577383576526557010503895406560826078403/2854495385411919762116571938898990272765493248;
Hf{3}(4,3) = (9744724398737937097135212199936*((22350236873591035419642750130871*u(2))/2535301200456458802993406410752 + (22350236873591035419642750130871*x(8))/2535301200456458802993406410752 + 68903552711818906676186879260929/79228162514264337593543950336))/(45758016894231850905448387765599*x(4)^2);
Hf{3}(8,3) = -96721252598033157730953235368645259990033528747/(51518946958518856104135573810492006426294091776*x(4));
Hf{3}(20,3) = -96721252598033157730953235368645259990033528747/(51518946958518856104135573810492006426294091776*x(4));
Hf{3}(3,4) = (9744724398737937097135212199936*((22350236873591035419642750130871*u(2))/2535301200456458802993406410752 + (22350236873591035419642750130871*x(8))/2535301200456458802993406410752 + 68903552711818906676186879260929/79228162514264337593543950336))/(45758016894231850905448387765599*x(4)^2);
Hf{3}(4,4) = -(19489448797475874194270424399872*x(3)*((22350236873591035419642750130871*u(2))/2535301200456458802993406410752 + (22350236873591035419642750130871*x(8))/2535301200456458802993406410752 + 68903552711818906676186879260929/79228162514264337593543950336))/(45758016894231850905448387765599*x(4)^3);
Hf{3}(8,4) = (96721252598033157730953235368645259990033528747*x(3))/(51518946958518856104135573810492006426294091776*x(4)^2);
Hf{3}(20,4) = (96721252598033157730953235368645259990033528747*x(3))/(51518946958518856104135573810492006426294091776*x(4)^2);
Hf{3}(8,7) = -32026325821508263713435834164223/10141204801825835211973625643008;
Hf{3}(20,7) = -32026325821508263713435834164223/10141204801825835211973625643008;
Hf{3}(1,8) = 20107228577383576526557010503895406560826078403/2854495385411919762116571938898990272765493248;
Hf{3}(3,8) = -96721252598033157730953235368645259990033528747/(51518946958518856104135573810492006426294091776*x(4));
Hf{3}(4,8) = (96721252598033157730953235368645259990033528747*x(3))/(51518946958518856104135573810492006426294091776*x(4)^2);
Hf{3}(7,8) = -32026325821508263713435834164223/10141204801825835211973625643008;
Hf{3}(19,8) = -32026325821508263713435834164223/10141204801825835211973625643008;
Hf{3}(8,19) = -32026325821508263713435834164223/10141204801825835211973625643008;
Hf{3}(20,19) = -32026325821508263713435834164223/10141204801825835211973625643008;
Hf{3}(1,20) = 20107228577383576526557010503895406560826078403/2854495385411919762116571938898990272765493248;
Hf{3}(3,20) = -96721252598033157730953235368645259990033528747/(51518946958518856104135573810492006426294091776*x(4));
Hf{3}(4,20) = (96721252598033157730953235368645259990033528747*x(3))/(51518946958518856104135573810492006426294091776*x(4)^2);
Hf{3}(7,20) = -32026325821508263713435834164223/10141204801825835211973625643008;
Hf{3}(19,20) = -32026325821508263713435834164223/10141204801825835211973625643008;


 Hf{4} = interval(sparse(20,20),sparse(20,20));



 Hf{5} = interval(sparse(20,20),sparse(20,20));

Hf{5}(1,1) = -x(4)*cos(x(1) + x(2));
Hf{5}(2,1) = -x(4)*cos(x(1) + x(2));
Hf{5}(4,1) = -sin(x(1) + x(2));
Hf{5}(1,2) = -x(4)*cos(x(1) + x(2));
Hf{5}(2,2) = -x(4)*cos(x(1) + x(2));
Hf{5}(4,2) = -sin(x(1) + x(2));
Hf{5}(1,4) = -sin(x(1) + x(2));
Hf{5}(2,4) = -sin(x(1) + x(2));


 Hf{6} = interval(sparse(20,20),sparse(20,20));

Hf{6}(1,1) = -x(4)*sin(x(1) + x(2));
Hf{6}(2,1) = -x(4)*sin(x(1) + x(2));
Hf{6}(4,1) = cos(x(1) + x(2));
Hf{6}(1,2) = -x(4)*sin(x(1) + x(2));
Hf{6}(2,2) = -x(4)*sin(x(1) + x(2));
Hf{6}(4,2) = cos(x(1) + x(2));
Hf{6}(1,4) = cos(x(1) + x(2));
Hf{6}(2,4) = cos(x(1) + x(2));


 Hf{7} = interval(sparse(20,20),sparse(20,20));



 Hf{8} = interval(sparse(20,20),sparse(20,20));



 Hf{9} = interval(sparse(20,20),sparse(20,20));



 Hf{10} = interval(sparse(20,20),sparse(20,20));



 Hf{11} = interval(sparse(20,20),sparse(20,20));



 Hf{12} = interval(sparse(20,20),sparse(20,20));



 Hf{13} = interval(sparse(20,20),sparse(20,20));



 Hf{14} = interval(sparse(20,20),sparse(20,20));



 Hf{15} = interval(sparse(20,20),sparse(20,20));



 Hf{16} = interval(sparse(20,20),sparse(20,20));



 Hf{17} = interval(sparse(20,20),sparse(20,20));



 Hf{18} = interval(sparse(20,20),sparse(20,20));

