clear 
clc
load('stateunc.mat')
load('statefeed.mat')
load('outputunc.mat')
load('outputfeed.mat')
load('modelbased.mat')
load('modelbasedunc.mat')
%% error 
figure;
subplot(2,1,1)
plot(T,e1b(:,1),T,e2b(:,1),T,e3b(:,1))
legend('modelbased','state feedback','output feedback')
title('e1')
subplot(2,1,2)
plot(T,e1b(:,2),T,e3b(:,2),T,e2b(:,2))
legend('modelbased','state feedback','output feedback')
title('e2')