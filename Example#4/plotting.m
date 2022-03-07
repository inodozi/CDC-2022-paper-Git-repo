function plotting(T_p,T_f,u_f,x_1,x_2,u_)

 j=[];
 j(1)=1;
 for i=1:T_p*T_f
 
 if u_f(i)==1
     j(i+1)=j(i);
 else
     j(i+1)=j(i)+1;
 end
 end 
 
t0=0:T_p*(T_f-1);
t=0:T_p*(T_f);

figure(6)
subplot(2,1,2)
plot(t0,u_,'LineWidth', 1)
xlabel('$\ell$','interpreter','latex')
ylabel('$u$','interpreter','latex')

subplot(2,2,1)
plotHarcColor(x_1,j',x_2,t')
grid on
xlabel('$x_1$','interpreter','latex')
ylabel('$x_2$','interpreter','latex')

subplot(2,2,2)
stairs(t0,u_f,'LineWidth', 1)
xlabel('$\ell$','interpreter','latex')
ylabel('$u_f$','interpreter','latex')

end