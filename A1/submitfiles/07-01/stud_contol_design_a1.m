 
%% embedded control systems course 2017
% Assignement # 1: Control over multicore architecture (CompSOC platform)
% Juan Valencia @ TUe
% Script to design the controller

%% CLEAR AND FORMAT
clc; format long;

%% Control application properties
reference = 1; % control reference objective
%desired_poles=computepoles(a1,a2,k1,k2);
C=16000;
 P=8000;
 
 TDM_TABLE_SIZE=4;
desired_poles = [0.8 0.8 0.8 0.85 0.9]; % desired closed-loop poles. TEMPLATE EXAMPLE: REMOVE THIS

%% CompSOC platform properties 
Fs  = 100e6; % global clock frequency: ML605 Board @ 100 MHz
TDM_FRAME = TDM_TABLE_SIZE * (C + P); % TDM frame duration: [slots]


% sampling periods: WARNING: DO NOT MODIFY hp 
hc = TDM_FRAME * (1/Fs) % controller sampling period

%hc = 1e-3; % TEMPLATE EXAMPLE: REMOVE THIS
hp = 100e-6; % plant simulation sampling period: WARNING: DO NOT MODIFY THIS VALUE

hc=ceil(hc/hp);
hc=hc*hp;

% sensor-to-actuator delay
delay_exact = ((P+C)*3) * (1/Fs); % exact sensor-to-actuator delay
%delay_exact = 2e-4; % TEMPLATE EXAMPLE: REMOVE THIS
delay_count = ceil(delay_exact/hp);
delay       = delay_count*hp
R=3*P/(TDM_TABLE_SIZE*(P+C));

% error messages
if (delay >= hc)
    error('ERROR: delay >= hc')
end

%% Physical system dynamics to control : continuous-time 
% System parameters
J1 = 3.75e-6;  % inertia [Kgm^2]
J2 = 3.75e-6;  % inertia [Kgm^2]
b  = 1e-5;     % friction coefficient [Nms/rad]
k  = 0.2656;   % torsional spring [Nm/rad]
d  = 3.125e-5; % torsional damping [Nms/rad]
Km = 4.4e-2;   % motor constant [Nm/A]
%````````````````````````````````````````````````````````````````
% State-space representation: fill your model (Ac, Bc)
Ac = [0 0 1 0 ; 0 0 0 1; -k/J1 k/J1 -(d+b)/J1 (d+b)/J1; k/J2 -k/J2  (d+b)/J2 -(d+b)/J2]; % TEMPLATE EXAMPLE: REMOVE THIS
Bc = [0; 0; Km/J2; 0]; % TEMPLATE EXAMPLE: REMOVE THIS
Cc = [1 0 0 0];
Dc = [0];
dim = length(Ac);

%% System discretization: for controller design
% @ controller side
sys_ss = ss(Ac,Bc,Cc,Dc);
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

% Augmented state variables
sysd_b0 = c2d(sys_ss, hc-delay); 
sysd_b1 = c2d(sys_ss, hc); 
B_0 = sysd_b0.b;
B_temp = sysd_b1.b;
B_1 = B_temp - B_0;

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

%% TO SIMULIK: WARNING : DO NOT CHANGE THIS PART OF THE CODE
% PLANT MODEL DISCRETIZATION @ 100 us
Ad_plant = [0.9996461   0.0003539   0.0000999   0.0000001; ...
            0.0003539   0.9996461   0.0000001   0.0000999; ...
            -7.0741539   7.0741539   0.9985473   0.0011861; ...
            7.0741539  -7.0741539   0.0011861   0.9985473];
Bd_plant = [0.0000586; 0.0000000; 1.1725500; 0.0006269];
Cd_plant = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Dd_plant = [0;0;0;0];        


        
        