clc 
clear all
close all

s = tf('s');
m=0.125;% Beam mass
L = 0.7; % Length of the beam
g=9.81;% Gravity
J= (m*L^2)/12; % Beam moment of inertia
K=0.1; % Gain motor
a0=K/J;
Bv=0.006; % Rigidity coefficient
a1=Bv/J;
G1=tf(a0,[1 a1 0]); % Transfer Fuction Angle 
G2=tf(-g,[1 0 0]); % Transfer Function Position

%Control first loop

%pidTuner(G1, 'PD');
Kp = 0.39211;
Kd = 1.0701;
C1 = Kp+Kd*s; % PD control
GT=feedback(C1*G1, 1);

%Control second loop

%pidTuner(Theta, 'PID');
Theta=(GT*G2);
Kp1 = -2.0865;
Ki1 = -0.34577;
Kd1 = -1.2438;
C2 = Kp1+Kd1*s+Ki1/s;
G=feedback(C2*Theta, 1);

step(G)
