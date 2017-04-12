

hc=0.01;
 delay=0.0081;
 %desired_poles=[0.8 0.8 0.05 0.00];
 %desired_poles=[x(1) x(2) x(3) x(4)];

desired_poles=[0.0231    0.8056    0.8283    0.0284]
% error messages
if (delay >= hc)
    error('ERROR: delay >= hc')
end

J = 3.2284E-6;
b = 3.5077E-6;
K = 0.0274;
R = 4;
L = 2.75E-6;

A = [0 1 0
    0 -b/J K/J
    0 -K/L -R/L];
B = [0 ; 0 ; 1/L];
C = [1 0 0];
dim = length(A);


sysc = ss(A,B,C,0);

T_period = hc;

%% System discretization: for controller design
% @ controller side
sys_ss = ss(A,B,C,0);
sys_d = c2d(sys_ss, hc, 'zoh');

% Controllability
co = ctrb(sys_d);
Controllability = rank(co);
if (Controllability  ~= dim) 
    warning('WARNING: System is uncontrollable!')
end;    

% Discrete matrices: for control design
Ad_controller_design = sys_d.a;
Bd_controller_design = sys_d.b;
Cd_controller_design = sys_d.c;
Dd_controller_design = sys_d.d;
phi=Ad_controller_design;
Gamma=Bd_controller_design;

% Augmented state variables
sysd_b0 = c2d(sys_ss, hc-delay); 
sysd_b1 = c2d(sys_ss, hc); 
B_0 = sysd_b0.b;
B_temp = sysd_b1.b;
B_1 = B_temp - B_0;
Gamma1=B_1;
Gamma0=B_0;

Aaug_controller = [Ad_controller_design  B_1; zeros(1,dim+1)];
Baug_controller = [B_0; 1];
Caug_controller = [Cd_controller_design 0];
Daug_controller = [0];

% conversion to single precision floating point
Aaug_controller = single(Aaug_controller);
Baug_controller = single(Baug_controller);
Caug_controller = single(Caug_controller);
Daug_controller = single(Daug_controller);

%% CONTROLLER DESIGN
% FEEDBACK GAIN
desired_poles = desired_poles';
K = -acker(Aaug_controller, Baug_controller, desired_poles); 
%K = [-3 -0.5 -0.005 -0.003 -0.5]; % TEMPLATE EXAMPLE: REMOVE THIS

% checking closed-loop poles with feedback controller
Acl = (Aaug_controller + Baug_controller*K);
poles_single=eigs(Acl);
if abs(eigs(Acl)) >= 1 
    warning('WARNING: Closed-loop poles are out of unit circle!')
end;    

% FEEDFORWARD GAIN
F = 1 / ( Caug_controller * ( (eye(dim+1) - Aaug_controller - (Baug_controller*K))^-1 ) * Baug_controller );
%F = 0.5; % TEMPLATE EXAMPLE: REMOVE THIS

        x1(2) = 0; x1(1) = 0;
x2(2) = 0; x2(1) = 0;
x3(2) = 0; x3(1) = 0;
input(2) = 0; input(1) = 0;
time(2) = T_period; time(1) = 0;
i = 2;
r = 3.1416;
for i=2:1/T_period
    
    u = K*[x1(i);x2(i);x3(i);input(i-1)] + r*F;
   
    xkp1 = phi*[x1(i);x2(i);x3(i)]+ Gamma1*input(i-1) + Gamma0*u;
    x1(i+1) = xkp1(1);
    x2(i+1) = xkp1(2);
    x3(i+1) = xkp1(3);
    input(i) = u;    
    time(i+1) = time(i) + T_period;   
end
plot(time, x1, 'r');
hold on;
%  
 plot(time(1:end-1), input, 'r');
mi=max(abs(input));
st=stepinfo(x1,time,r);
st=st.SettlingTime;
if(isnan(st))
   st=100; 
end
y(1)=mi;
y(2)=st;
        