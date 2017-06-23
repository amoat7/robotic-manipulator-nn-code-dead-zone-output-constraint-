function [xdot]= unconstrained(t,x)
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
%ddqd(2)=-a2*(freq^2)*cos(freq*t);
%%
W=(x(7:22));
W_tau=(x(23:38));
%% 
m1=1; m2=1; l1=0.8; l2=0.7; %mass of robot arms and length of links
k2=10; %controller gains
k1=10;
%% computation of the tracking errors
e=[x(1) x(2)]-qd; 
alp1=-k1*e+dqd; 
e2=[x(3) x(4)]-alp1;  e=e';
dalp1=-k1*([x(3) x(4)]-dqd)+ddqd;
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
 %% computation of G matrix
g1=m1*l1*9.8*cos(x(1))+m2*9.8*(l2*cos(x(1)+x(2))+l1*cos(x(1)));
g2=m2*l2*9.8*cos(x(1)+x(2));
G = [g1;g2];
%% calculating control torques
torque=[x(5);x(6)];
%% nn parameters
sigma = 0.02;
sigma_tau = 0.02;
gamm = 100;
gamm_tau = 100;

variance = 100;
variance_tau = 150;
Node = 2^4;
Node_tau = 2^4;
k = 1;
Mu=zeros(4,16);
for i1 = 0.5*(-1:2:1)
    for i2 = 0.5*(-1:2:1)
        for i3 = 0.5*(-1:2:1)
            for i4 = 0.5*(-1:2:1)
                Mu(:,k) = [i1,i2,i3,i4];
                k = k+1;
            end
        end
    end
end
x1=[x(1) x(2)];
x2=[x(3) x(4)];
Z=[x1' x2' alp1' dalp1']';
Z_tau=[torque x1' x2' alp1']';
S=zeros(Node,1);
S_tau=zeros(Node,1);
for i =1:Node
    S(i,1)=exp(-(Z(:,1)-Mu(:,i))'*(Z(:,1)-Mu(:,i))/variance);
    S(i,2)=exp(-(Z(:,2)-Mu(:,i))'*(Z(:,2)-Mu(:,i))/variance);
end
for i =1:Node
    S_tau(i,1)=exp(-(Z_tau(:,1)-Mu(:,i))'*(Z_tau(:,1)-Mu(:,i))/variance);
    S_tau(i,2)=exp(-(Z_tau(:,2)-Mu(:,i))'*(Z_tau(:,2)-Mu(:,i))/variance);
end
torque=-k2.*e2'-e+(W'*S + W_tau'*S_tau)';
% updating laws
 dW = -gamm.*( S*e2' + sigma*W );
 dW_tau = -gamm_tau.*( S_tau*e2' + sigma_tau*W_tau );
%% 
%D = deadzone('zeroInterval',[-4.5 2.5]);
%torque = evaluate(D, torque);
%% robot arm dynamics
qdotdot= (M)\(torque-G-C-[3;3.4]);
xdot=[x(3);x(4);qdotdot(1);qdotdot(2);
    (torque(1)-x(5));%%ode will not integrate this, output only
    (torque(2)-x(6));%%ode will not integrate this, ouput only
    dW(1:16); dW_tau(1:16)
    ];
