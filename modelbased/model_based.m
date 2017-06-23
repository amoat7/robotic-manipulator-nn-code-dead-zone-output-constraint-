function [xdot]= model_based(t,x)
%% desired trajectory
a1=0.1; a2=0.1; per=2; freq=pi/per; %amplitudes and period
qd(1)=a1*sin(freq*t);  
qd(2)=a1*sin(freq*t);
%qd(2)=a2*cos(freq*t);  
dqd(1)=a1*freq*cos(freq*t); 
%dqd(2)=-a2*freq*sin(freq*t);
dqd(2)=a1*freq*cos(freq*t); 
ddqd(1)=-a1*(freq^2)*sin(freq*t);
ddqd(2)=-a1*(freq^2)*sin(freq*t);
%ddqd(2)=-a2*(freq^2)*cos(freq*t);;
dt=1.5*(1-exp(-0.28.*t)).*(sin(0.5*pi.*t));
D = deadzone('ZeroInterval',[-4.5, 2.5]);
%% 
m1=1; m2=1; l1=0.8; l2=0.7; %mass of robot arms and length of links
k2=5; %controller gains
k1=5;
b=0.005;
%% 
%% computation of the tracking errors
e=[x(1) x(2)]-qd; 
alp1=-(b^2-(e*e'))*k1*e+dqd; 
e2=[x(3) x(4)]-alp1;  e=e';
dalp1=ddqd;
%% computation of mass inertia matrix
m11=(m1+m2)*l1^2+m2*l2^2+2*m2*l1*l2*cos(x(2));
m12=m2*l2^2+m2*l1*l2*cos(x(2));
m21=m12;
m22=m2*l2^2;
M = [m11 m12; m21 m22];   
%% computation of C matrix
c11=-m1*l1*l2*x(4)*sin(x(2));
c12=-m2*l1*l2*(x(3)+x(4))*sin(x(2));
c21=m2*l1*l2*x(3)*sin(x(2));
c22=0;
C=[c11 c12; c21 c22]*[x(3);x(4)];
C1=[c11 c12; c21 c22];
 %% computation of G matrix
g1=m1*l1*9.8*cos(x(1))+m2*9.8*(l2*cos(x(1)+x(2))+l1*cos(x(1)));
g2=m2*l2*9.8*cos(x(1)+x(2));
G = [g1;g2];
%% calculating control torques
torque=-[e(1)/(b^2-e(1)^2);e(2)/(b^2-e(2)^2)]-k2.*e2'+C1*alp1'+G+M*dalp1'+dt;

torque(1) = evaluate(D,torque(1));
torque(2)=evaluate(D,torque(2));
%% robot arm dynamics
qdotdot= (M)\(torque-G-C-dt);
xdot=[x(3);x(4);qdotdot(1);qdotdot(2);
    (torque(1)-x(5));%%ode will not integrate this, output only
    (torque(2)-x(6));%%ode will not integrate this, ouput only 
    ];
