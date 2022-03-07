clear
close all
clc
%% flow map parameters
 A_f=[1/2 -sqrt(3)/2
     sqrt(3)/2 1/2];
B_f=[0
    01];
%% jump map parameters
A_g=.8*[1/2 sqrt(3)/2
    -sqrt(3)/2 1/2];
 B_g=[0
    01];

%% jump and flow set  parameters
h=[1, 0];
sigma=0.1;

%% feasiblity set
x_1max=10;
x_1min=-10;

x_2max=10;
x_2min=-10;

u_max=1;
u_min=-1;

%% finding max and min of flow map
M11=x_1max;
M12=x_2max;

m11=x_1min;
m12=x_2min;

%% finding max and min of jump map
M2=u_max;

m2=u_min;

%% max and min on h(x)
cc8=[h(1);h(2)]>=0;
M3=h(1)*(cc8(1)*x_1max+(1-cc8(1))*x_1min)+h(2)*(cc8(2)*x_2max+(1-cc8(2))*x_2min);

cc9=[h(1);h(2)]<=0;
m3=h(1)*(cc9(1)*x_1max+(1-cc9(1))*x_1min)+h(2)*(cc9(2)*x_2max+(1-cc9(2))*x_2min);

%% relation between x(i)=A_f*z_1(i-1)+B_f*z_2(i-1)+A_g(x(i-1)-z_1(i-1))+B_g(u(i-1)-z_2(i-1))
A_fg=A_f-A_g;
B_fg=B_f-B_g;
Z=zeros(2,1);

AA=[A_fg B_fg Z B_g];
H1=h(1,1)*AA(1,:)+h(1,2)*AA(2,:);

A_10=[zeros(6,5)
    -AA
    zeros(1,5)
    AA
    zeros(1,5)
    -H1
    H1
    AA
    -AA
    zeros(2,5)];

%% relation of current parameters
A0=[1 0 0 -M11 0 
    0 1 0 -M12 0
    0 0 1 -M2 0
    -1 0 0 m11 0
    0 -1 0 m12 0
    0 0 -1 m2 0
    1 0 0 -m11 0
    0 1 0 -m12 0
    0 0 1 -m2 -1
    -1 0 0 M11 0
    0 -1 0 M12 0
    0 0 -1 M2 1
    0 0 0 (m3+sigma) 0
    0 0 0 (M3-sigma) 0
    0 0 0 0 0
    0 0 0 0 0
    0 0 0 0 0
    0 0 0 0 0
    0 0 0 0 1
    0 0 0 0 -1];
%% writting A and b
ZZ=zeros(20,5);

A=[A0 ZZ
    A_10 A0];
%% cost function parameters
Q_c=0.1*eye(2);
Q_d=0.3*eye(2);
R_c=0.2;
R_d=0.1;
Q_cda=1/2*(Q_c-Q_d)*AA;
w=[0 0 0 0 0
    0 0 0 0 1/2*(R_c-R_d)
    1/2*(R_c-R_d) 0 0 0 0
    0 0 0 0 0
    0 0 0 0 R_d];
B=[Q_cda
    zeros(3,5)];

S1=zeros(10,10);
S2=zeros(10,1);
%% initial value
x0=zeros(10,1);

x1=-1;
x2=-1;
T_f=20; % simulation time
T_p=2; % prediction horizon
x_1=x1;
x_2=x2;
u_f=[];
u_=[];
 for i=1:T_f

     
%% b for next steps that x(i) is written as sumation of z_1 and z_2 so we dont need them in b
x=[x1
    x2];
m1=[m11
    m12];
M1=[M11
    M12];
xmax=[x_1max
    x_2max];
xmin=[x_1min
    x_2min];
b1=[zeros(6,1)
    A_g*x-m1
    -m2
    -A_g*x+M1
    M2
    h(1,1)*A_g(1,:)*x+h(1,2)*A_g(2,:)*x+sigma
    -h(1,1)*A_g(1,:)*x-h(1,2)*A_g(2,:)*x+M3
    -A_g*x+xmax
    A_g*x-xmin
    u_max
    -u_min];
%% b for current steps 
 b0=[zeros(6,1)
    x1-m11
    x2-m12
    -m2
    -x1+M11
    -x2+M12
    M2
    0+h(1,1)*x1+h(1,2)*x2+sigma
    M3-h(1,1)*x1-h(1,2)*x2
    0
    0
    0
    0
    u_max
    -u_min];
bb=[b0
    b1];
%% binary variabels
ivar=[4,9];
options = [];
miqp(S1,S2,A,bb,[],[],ivar,[],[],x0,options);

x_11=AA*ans(1:5)+A_g*x;
x_22=AA*ans(6:10)+A_g*x_11;

x_12=[x_11 x_22]';
     
x1=x_22(1);
x2=x_22(2);    
   
x_1=[x_1; x_12(:,1)];
x_2=[x_2; x_12(:,2)];

u_fi=[ans(4),ans(9)];
u_f=[u_f,u_fi] ;
u_i=[ans(5),ans(10)];
u_=[u_,u_i] ;  
x0=ans;
 end
 %% ploting
 j=[];
 j(1)=1;
 for i=1:T_p*T_f
 
 if u_f(i)==1
     j(i+1)=j(i);
 else
     j(i+1)=j(i)+1;
 end
 end
xx=[x_1 x_2];
t0=0:T_p*(T_f-.5);
t=0:T_p*T_f;

figure(1)
stairs(t0,u_f,'LineWidth', 1.5)
xlabel('$\ell$','interpreter','latex')
ylabel('$\rho(u_{f1},u_{f2})$','interpreter','latex')
ylim([-.01 1.01])
set(get(gca,'ylabel'),'rotation',0)


figure(2)
plotHarcColor(x_1,j',x_2,t')
grid on
xlabel('$x_1$','interpreter','latex')
ylabel('$x_2$','interpreter','latex')
