figure;
Y2n=Y(:,7:22);
Y2m=Y(:,23:38);
for i=1:length(T)
    Y2norm(i,:)=norm(Y2n(i,:));
    Y2tau(i,:)=norm(Y2m(i,:));
end
subplot(2,1,1)
plot(T,Y2norm)
legend('state feedback')
title('norm of RBF neural network')
subplot(2,1,2)
plot(T,Y2tau)
legend('state feedback')
title('norm of RBF neural network')