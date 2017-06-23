clear
clc
tic;
t0=0; tf=20;
x0=zeros(38,1); t=linspace(t0,tf,300);
[T,Y]=ode23('unconstrained',t,x0);
%% desired trajectory
a1=0.1; a2=0.1; per=2; freq=pi/per; %amplitudes and period
qd(:,1)=a1*sin(freq*T);  
%qd(:,2)=a2*cos(freq*T);
qd(:,2)=a1*sin(freq*T);
%% plot of torques;
figure
plot(T,Y(:,5),'r',T,Y(:,6),'b');
title('torque')
legend('t1','t2');
%% plot error
figure; 
error(:,1)=qd(:,1)-Y(:,1);
error(:,2)=Y(:,2)-qd(:,2);
plot(T,error);
title('error');
%% plot of actual trajectory and desired trajectory
figure;
subplot(2,1,1)
plot(T,Y(:,1),'r',T,qd(:,1),'b');
title('desired trajectory qd1 and actual trajectory q1');
subplot(2,1,2)
plot(T,Y(:,2),'r',T,qd(:,2),'b');
title('desired trajectory qd1 and actual trajectory q2');

toc;