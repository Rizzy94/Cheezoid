%% PATH SIMPLIFICATION TESTING

tic
p = PathPlanning([20 20], [85 85] ,[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105],1)
toc
pathShortening(p,size(p,1),[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105])
