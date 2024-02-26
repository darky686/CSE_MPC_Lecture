M = 0;
m = 0;
b = 0;
I = 0;
g = 0;
l = 0;


A = 0;
B = 0;
C = 0;
D = 0;

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x'; 'phi'};



%GET STATE SPACE SYSTEM
sys_ss = 0; %F(params)


%DISCRETIZATION OF STATE SPACE SYSTEM
Ts = 1/100;

sys_d = 0; %continueous to discrete F(sys_ss)

%TEST Controllability and observability
co = 0;%F(sys_d);
ob = 0;%F(sys_d);

controllability = rank(co)
observability = rank(ob)

%%LQR 

A = sys_d.a;
B = sys_d.b;
C = sys_d.c;
D = sys_d.d;
Q = C'*C
R = 1;

%DO LQR
[K] = 0; %F(SYS,Q,R)

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'};

%STATE SPACE OF THE SYSTEM
sys_cl = 0; %F(SYSc)

t = 0:0.01:5;
r =0.2*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with Digital LQR Control')