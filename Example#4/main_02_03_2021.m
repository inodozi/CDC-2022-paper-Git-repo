clear
close all
clc
%% flow map parameters
 A_f=[1/2 -.86
     .86 1/2];
B_f=[0
    1];
%% jump map parameters
A_g=[.4 .69
    -.69 .4];
 B_g=[0
    1];
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
xmax=[x_1max
    x_2max];
xmin=[x_1min
    x_2min];
%% finding max and min of flow map
M11=x_1max;
M12=x_2max;
m11=x_1min;
m12=x_2min;
m1=[m11
    m12];
M1=[M11
    M12];
%% finding max and min of jump map
M2=u_max;
m2=u_min;
%% max and min on h(x)
cc8=[h(1);h(2)]>=0;
M3=h(1)*(cc8(1)*x_1max+(1-cc8(1))*x_1min)+h(2)*(cc8(2)*x_2max+(1-cc8(2))*x_2min);
cc9=[h(1);h(2)]<=0;
m3=h(1)*(cc9(1)*x_1max+(1-cc9(1))*x_1min)+h(2)*(cc9(2)*x_2max+(1-cc9(2))*x_2min);
%% calculating AA for x(\ell)=AA*[z(\ell-1),u_f(\ell-1),u(\ell-1)]
AA=AA(A_f,A_g,B_f,B_g);
%% calculating A for AX<b
A=F1(AA,A_g,h,M11,M12,M2,M3,m11,m12,m2,m3,sigma);
%% cost function parameters
Q_c=2e-1*eye(2);
Q_d=1e-1*eye(2);
P=1e-2*eye(2);
R_c=1e-1;
R_d=1e-1;

%% initial value
x0=zeros(20,1);
x1=-1;
x2=-1;
T_f=10; % simulation time
T_p=1; % control horizon
x_1=x1;
x_2=x2;
u_f=[];
u_=[];
 for i=1:T_f
%% b for next steps that x(i) is written as sumation of z_1 and z_2 so we dont need them in b
x=[x1
    x2];

 [S1,S2]=costfunction(AA,Q_c,A_g,Q_d,R_d,R_c,P,x);

%%
     bb=b(A_g,x,x1,x2,m2,M11,M12,M2,h,sigma,M3,xmax,xmin,m11,m12,u_max,u_min);
%% binary variabels
ivar=[4,9,14,19];
options = [];
miqp(S1,S2,A,bb,[],[],ivar,[],[],x0,options);

%% t_c=1
x_11=AA*ans(1:5)+A_g*x;

x_12=[x_11]';     
   x1=x_11(1);
  x2=x_11(2);    
  x_1=[x_1; x_12(:,1)];
 x_2=[x_2; x_12(:,2)];
  u_fi=[ans(4)];
u_f=[u_f,u_fi] ;
 
   u_i=[ans(5)];
u_=[u_,u_i] ;


 
   x0=ans;
 end
 %% ploting
plotting(T_p,T_f,u_f,x_1,x_2,u_)