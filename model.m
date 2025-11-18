%% DC Converter - Small-Signal Averaged Model
% model.m
% State-space averaging and small-signal linearization for a boost converter
% Author: <Reza Khodadadi>
% Date: <2025-11-18>

clc; clear; close all;

% -----------------------
% Parameters
% -----------------------
R = 5; 
VIN = 50;
rin = 0.1;
L = 400e-6;
rL = 0.1;
C = 100e-6;
rC = 0.05;
VD = 0.7;
rD = 0.01;
rds = 0.1;
D = 0.41;

% -----------------------
% Symbolic definitions
% -----------------------
syms iL vC vin vD d

% ON-state (switch closed) dynamics
M1 = (-(rin+rds+rL+(R*rC/(R+rC)))*iL -R/(R+rC)*vC+vin)/L;
M2 = (R/(R+rC)*iL  -1/(R+rC)*vC)/C;
vO1 = R*(rC/(rC+R)*iL +1/(R+rC)*vC);

% OFF-state (switch open) dynamics
M3 = (-(rD+rL+R*rC/(R+rC))*iL -R/(R+rC)*vC -vD)/L;
M4 = (R/(R+rC)*iL  -1/(R+rC)*vC)/C;
vO2 = R*(rC/(rC+R)*iL +1/(R+rC)*vC);

% Averaged model (in-state equations)
MA1 = simplify(d*M1 + (1-d)*M3);
MA2 = simplify(d*M2 + (1-d)*M4);
vO  = simplify(d*vO1 + (1-d)*vO2);

% Substitute DC operating point values
MA_DC_1 = subs(MA1, [vin vD d], [VIN VD D]);
MA_DC_2 = subs(MA2, [vin vD d], [VIN VD D]);

% Solve for steady-state (DC) operating point
DC_SOL = solve(MA_DC_1 == 0, MA_DC_2 == 0, 'Real', true);

% Convert solutions to numeric
IL = double(DC_SOL.iL);
VC = double(DC_SOL.vC);

% Linearize (Jacobian) around operating point
A11 = subs(simplify(diff(MA1, iL)), [iL vC d vD], [IL VC D VD]);
A12 = subs(simplify(diff(MA1, vC)), [iL vC d vD], [IL VC D VD]);
A21 = subs(simplify(diff(MA2, iL)), [iL vC d vD], [IL VC D VD]);
A22 = subs(simplify(diff(MA2, vC)), [iL vC d vD], [IL VC D VD]);
A = double([A11 A12; A21 A22]);

B11 = subs(simplify(diff(MA1, vin)), [iL vC d vD vin], [IL VC D VD VIN]);
B12 = subs(simplify(diff(MA1, d)),   [iL vC d vD vin], [IL VC D VD VIN]);
B21 = subs(simplify(diff(MA2, vin)), [iL vC d vD vin], [IL VC D VD VIN]);
B22 = subs(simplify(diff(MA2, d)),   [iL vC d vD vin], [IL VC D VD VIN]);
B = double([B11 B12; B21 B22]);

CC1 = subs(simplify(diff(vO, iL)), [iL vC d vD], [IL VC D VD]);
CC2 = subs(simplify(diff(vO, vC)), [iL vC d vD], [IL VC D VD]);
CC = double([CC1 CC2]);

DD1 = subs(simplify(diff(vO, vin)), [iL vC d vD vin], [IL VC D VD VIN]);
DD2 = subs(simplify(diff(vO, d)),   [iL vC d vD vin], [IL VC D VD VIN]);
DD = double([DD1 DD2]);

% Create state-space and transfer functions
sys = ss(A, B, CC, DD);
H = tf(sys);

vo_vin = H(1, 1);
vo_d   = H(1, 2);

% Display results
disp('Operating point:');
fprintf(' IL = %.6f A\n VC = %.6f V\n', IL, VC);

disp('State-space A matrix:'); disp(A);
disp('State-space B matrix:'); disp(B);
disp('Output matrix C:');      disp(CC);
disp('Feedthrough D:');       disp(DD);

disp('Transfer functions:');
vo_vin, vo_d

% Optional Bode plots
figure;
bode(vo_d); grid on; title('Bode of vo/d (control-to-output)');
figure;
bode(vo_vin); grid on; title('Bode of vo/vin (input-to-output)');
