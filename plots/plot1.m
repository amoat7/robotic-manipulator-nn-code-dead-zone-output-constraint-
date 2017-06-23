clear 
clc
load('stateunc.mat')
load('statefeed.mat')
load('outputunc.mat')
load('outputfeed.mat')
load('modelbased.mat')
load('modelbasedunc.mat')
%% tracking
figure;
subplot(2,1,1)
plot(T,0.105*ones(size(T)),'--',T,-0.105*ones(size(T)),'--',T,Y1(:,1),T,Y2(:,1),T,Y2(:,1),T,qd(:,1));
title('desired trajectory qd1 and actual trajectory q1');
legend('upper bound of output constraint','lower bound of output constraint','modelbased','state feedback','output feedback','desired trajectory')
subplot(2,1,2)
plot(T,0.105*ones(size(T)),'--',T,-0.105*ones(size(T)),'--',T,Y1(:,2),T,Y2(:,2),T,Y2(:,2),T,qd(:,2));
title('desired trajectory qd2 and actual trajectory q2');
legend('upper bound of output constraint','lower bound of output constraint','modelbased','state feedback','output feedback','desired trajectory')
%% error 
plot(T,e1(:,1),T,e2(:,1),T,e3(:,1))
legend('modelbased','state feedback','output feedback')
title('e1')
plot(T,e1(:,2),T,e2(:,2),T,e3(:,2))
legend('modelbased','state feedback','output feedback')
title('e2')
%% torques
figure;
subplot(2,1,1)
plot(T,Y1(:,5),T,Y2(:,5),T,Y3(:,5))
legend('modelbased','state feedback','output feedback')
title('t1')
subplot(2,1,2)
plot(T,Y1(:,6),T,Y2(:,6),T,Y3(:,6))
legend('modelbased','state feedback','output feedback')
title('t2')
%% norms
figure;
Y2n=Y2(:,7:22);
Y3n=Y3(:,7:22);
Y2m=Y2(:,23:38);
Y3m=Y3(:,23:38);
for i=1:length(T)
    Y2norm(i,:)=norm(Y2n(i,:),1);
    Y2tau(i,:)=norm(Y2m(i,:),1);
    Y3norm(i,:)=norm(Y3n(i,:),1);
    Y3tau(i,:)=norm(Y3m(i,:),1);
end
subplot(2,1,1)
plot(T,Y2norm,T,Y3norm)
legend('state feedback','output feedback')
title('norm of RBF neural network')
subplot(2,1,2)
plot(T,Y2tau,T,Y3tau)
legend('state feedback','output feedback')
title('norm of RBF neural network')
%% unconstrained tracking a
figure;
subplot(2,1,1)
plot(T,0.105*ones(size(T)),'--',T,-0.105*ones(size(T)),'--',T,Y4a(:,1),T,Y5a(:,1),T,Y6a(:,1),T,qd(:,1));
title('desired trajectory qd1 and actual trajectory q1');
legend('upper bound of output constraint','lower bound of output constraint','model based','state feedback','output feedback','desired trajectory')
subplot(2,1,2)
plot(T,0.105*ones(size(T)),'--',T,-0.105*ones(size(T)),'--',T,Y4a(:,2),T,Y5a(:,2),T,Y6a(:,2),T,qd(:,2));
title('desired trajectory qd2 and actual trajectory q2');
legend('upper bound of output constraint','lower bound of output constraint','model based','state feedback','output feedback','desired trajectory')
%% unconstrained error a
figure;
subplot(2,1,1)
plot(T,e5a(:,1),T,e6a(:,1))
legend('state feedback','output feedback')
title('e1')
subplot(2,1,2)
plot(T,e5a(:,2),T,e6a(:,2))
legend('state feedback','output feedback')
title('e2')
%% unconstrained tracking c
figure;
subplot(2,1,1)
plot(T,0.105*ones(size(T)),'--',T,-0.105*ones(size(T)),'--',T,Y4c(:,1),T,Y5c(:,1),T,qd(:,1));
title('desired trajectory qd1 and actual trajectory q1');
legend('upper bound of output constraint','lower bound of output constraint','state feedback','output feedback','desired trajectory')
subplot(2,1,2)
plot(T,0.105*ones(size(T)),'--',T,-0.105*ones(size(T)),'--',T,Y4c(:,2),T,Y5c(:,2),T,qd(:,2));
title('desired trajectory qd2 and actual trajectory q2');
legend('upper bound of output constraint','lower bound of output constraint','state feedback','output feedback','desired trajectory')
%% unconstrained error c
figure;
e1a=qd(:,1)-Y4c(:,1);
e2a=-qd(:,2)+Y4c(:,2);
e3a=qd(:,1)-Y5c(:,1);
e4a=-qd(:,2)+Y5c(:,2);
subplot(2,1,1)
plot(T,e1a,T,e3a)
legend('state feedback','output feedback')
title('error1')
subplot(2,1,2)
plot(T,e2a,T,e4a)
legend('state feedback','output feedback')
title('error2')
%% unconstrained tracking d
figure;
subplot(2,1,1)
plot(T,0.105*ones(size(T)),'--',T,-0.105*ones(size(T)),'--',T,Y4d(:,1),T,Y5d(:,1),T,qd(:,1));
title('desired trajectory qd1 and actual trajectory q1');
legend('upper bound of output constraint','lower bound of output constraint','state feedback','output feedback','desired trajectory')
subplot(2,1,2)
plot(T,0.105*ones(size(T)),'--',T,-0.105*ones(size(T)),'--',T,Y4d(:,2),T,Y5d(:,2),T,qd(:,2));
title('desired trajectory qd2 and actual trajectory q2');
legend('upper bound of output constraint','lower bound of output constraint','state feedback','output feedback','desired trajectory')


