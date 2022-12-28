%% Clear the environment and the command line
clear;
clc;
close all;

%% Add the directory containing relevant functions to the path variables
addpath('./Standford-functions/');

%% Define the input parameters 
disp("Please enter the joint variables (in degree):")
theta1 = InputParameter('θ1 (-160~160): ', -160, 160);
theta2 = InputParameter('θ2 (-125~125): ', -120, 125);
d3 = InputParameter('d3 (-30~30): ', -30, 30);
theta4 = InputParameter('θ4 (-140~140): ', -140, 140);
theta5 = InputParameter('θ5 (-100~100): ', -100, 100);
theta6 = InputParameter('θ6 (-260~260): ', -260, 260);

% DH model 
d = [0 6.375 d3 0 0 0];
alpha = [-90 90 0 -90 90 0];
a = [0 0 0 0 0 0];
theta = [theta1 theta2 0 theta4 theta5 theta6];

%% simulate
% Global Reference frame
Trans = TransMatrices(0,0,0,0);

for i = 1:6
    Trans = TransMatrices(alpha(i), a(i), d(i), theta(i), Trans);
end

disp('\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\');
disp('(n, o, a, p) :')
disp(Trans);
[x, y, z, A, B, C] = Euler_To_Cartesian(Trans);
disp('(x, y, z, ϕ, θ, ψ) :');
fprintf("  %.6f %.6f %.6f %.6f %.6f %.6f\n\n", x, y, z, A, B, C);
