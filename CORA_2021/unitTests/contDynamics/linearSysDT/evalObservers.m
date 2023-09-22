
clear all;

load IH_FRadA_R2020a;

IHsups_20 = IHsups;
IHinfs_20 = IHinfs;

load IH_FRadA_R2021a;

IHsups_21 = IHsups;
IHinfs_21 = IHinfs;

clear IHinfs IHsups

% norm of infs diff
norminfs = vecnorm(IHinfs_20 - IHinfs_21,2);
normsups = vecnorm(IHsups_20 - IHsups_21,2);

figure; hold on;
title("lin tank FRad-B");
hinfs = plot(norminfs);
hsups = plot(normsups);
legend([hinfs,hsups],'Norm Diff Inf','Norm Diff Sup');
close;