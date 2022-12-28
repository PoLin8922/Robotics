%% Clear the environment and the command line
clear;
clc;
close all;

%% Add the directory containing relevant functions to the path variables
addpath('./Standford-functions/');

%% Define waypoints and parameters
t_a = 0;
t_b = 0.5;
t_c = 1;
t_sampling = 0.002;
t_acc = 0.2;

A = [ 0  0 -1  10
     -1  0  0  20
      0  1  0  30
      0  0  0   1];
[x1, y1, z1, phi1, theta1, psi1] = Euler_To_Cartesian(A);
  
 B = [ 1  0  0 30
       0  1  0 20
       0  0  1 10
       0  0  0  1];
[x2, y2, z2, phi2, theta2, psi2] = Euler_To_Cartesian(B);
  
 C = [ 0 -1  0 -10
       0  0  1 -20
      -1  0  0 -30
       0  0  0   1];
[x3, y3, z3, phi3, theta3, psi3] = Euler_To_Cartesian(C);

%% Path planning
% straight line portion (A to B)
t1 = t_a:t_sampling:t_b-t_acc;
size1 = (t_b-t_acc-t_a)/t_sampling +1;
[px_s1, vx_s1, ax_s1]  = Straight_line_portion(x1, x2, t_a, t_b, t1);
[py_s1, vy_s1, ay_s1] = Straight_line_portion(y1, y2, t_a, t_b, t1);
[pz_s1, vz_s1, az_s1] = Straight_line_portion(z1, z2, t_a, t_b, t1);
phi_s1 = Straight_line_portion(phi1, phi2, t_a, t_b, t1);
theta_s1 = Straight_line_portion(theta1, theta2, t_a, t_b, t1);
psi_s1 = Straight_line_portion(psi1, psi2, t_a, t_b, t1);
% z direction in ground frame (for drawing arrow of 3D Cartesian path
u1 = cosd(phi_s1).*sind(theta_s1);
v1 = sind(phi_s1).*sind(theta_s1);
w1 = cosd(theta_s1);

% transition portion
t2 = t_b-t_acc:t_sampling:t_b+t_acc;
[px_t, vx_t, ax_t] = Transition_portion(px_s1(size1), x2, x3, t_b, t_c, t2-t_b);
[py_t, vy_t, ay_t] = Transition_portion(py_s1(size1), y2, y3, t_b, t_c, t2-t_b);
[pz_t, vz_t, az_t] = Transition_portion(pz_s1(size1), z2, z3, t_b, t_c, t2-t_b); 
phi_t = Transition_portion(phi_s1(size1), phi2, phi3, t_b, t_c, t2-t_b);
theta_t = Transition_portion(theta_s1(size1), theta2, theta3, t_b, t_c, t2-t_b);
psi_t = Transition_portion(psi_s1(size1), psi2, psi3, t_b, t_c, t2-t_b);
% z direction in global frame (for drawing arrow of 3D Cartesian path
u2 = cosd(phi_t).*sind(theta_t);
v2 = sind(phi_t).*sind(theta_t);
w2 = cosd(theta_t);

% straight line portion (B to C)
t3 = t_b+t_acc:t_sampling:t_c;
[px_s2, vx_s2, ax_s2] = Straight_line_portion(x2, x3, t_b, t_c, t3-t_b);
[py_s2, vy_s2, ay_s2] = Straight_line_portion(y2, y3, t_b, t_c, t3-t_b);
[pz_s2, vz_s2, az_s2] = Straight_line_portion(z2, z3, t_b, t_c, t3-t_b);
phi_s2 = Straight_line_portion(phi2, phi3, t_b, t_c, t3-t_b);
theta_s2 = Straight_line_portion(theta2, theta3, t_b, t_c, t3-t_b);
psi_s2 = Straight_line_portion(psi2, psi3, t_b, t_c, t3-t_b);
% z direction in ground frame (for drawing arrow of 3D Cartesian path
u3 = cosd(phi_s2).*sind(theta_s2);
v3 = sind(phi_s2).*sind(theta_s2);
w3 = cosd(theta_s2);

%% Draw path
% position configuration
figure(1);
subplot(3,1,1); plot(t1,px_s1,'b', t2,px_t,'b', t3,px_s2,'b' ); title('Position of x');
subplot(3,1,2); plot(t1,py_s1,'b', t2,py_t,'b', t3,py_s2,'b' ); title('Position of y'); ylabel('position (cm)');
subplot(3,1,3); plot(t1,pz_s1,'b', t2,pz_t,'b', t3,pz_s2,'b' ); title('Position of z'); xlabel('time (s)') 

% velocity configuration
figure(2);
subplot(3,1,1); plot(t1,vx_s1,'b', t2,vx_t,'b', t3,vx_s2,'b' ); title('Velocity of x');
subplot(3,1,2); plot(t1,vy_s1,'b', t2,vy_t,'b', t3,vy_s2,'b' ); title('Velocity of y'); ylabel('velocity (cm/s)');
subplot(3,1,3); plot(t1,vz_s1,'b', t2,vz_t,'b', t3,vz_s2,'b' ); title('Velocity of z'); xlabel('time (s)') 

% acceleration configuration
figure(3);
subplot(3,1,1); plot(t1,ax_s1,'b', t2,ax_t,'b', t3,ax_s2,'b' ); title('Acceleration of x');
subplot(3,1,2); plot(t1,ay_s1,'b', t2,ay_t,'b', t3,ay_s2,'b' ); title('Acceleration of y'); ylabel('acceleration (cm/s^2)');
subplot(3,1,3); plot(t1,az_s1,'b', t2,az_t,'b', t3,az_s2,'b' ); title('Acceleration of z'); xlabel('time (s)') 

% Cartesian path
figure(4);
% draw position motion of end-effector  
plot3(px_s1, py_s1, pz_s1,'b', px_t, py_t, pz_t,'b',px_s2, py_s2, pz_s2, 'b', 'linewidth', 1.5);
title('3D Cartesian path'); xlabel('x (cm)');  ylabel('y (cm)');  zlabel('z (cm)'); 
grid on;
hold on;
% draw orientation motion of end-effector
q1 = quiver3(px_s1, py_s1, pz_s1, u1*5, v1*5, w1*5, 0);
q1.Color = 'c';
q1.MaxHeadSize = 0.1;
q2 = quiver3(px_t, py_t, pz_t, u2*5, v2*5, w2*5, 0);
q2.Color = 'c';
q2.MaxHeadSize = 0.1;
q3 = quiver3(px_s2, py_s2, pz_s2, u3*5, v3*5, w3*5, 0);
q3.Color = 'c';
q3.MaxHeadSize = 0.1;
% draw point A, B and C 
plot3(10, 20, 30,'x', 30, 20, 10,'x', -10, -20, -30, 'x', 'color', 'r');
text(11, 20, 30,'A (10, 20, 30)');
text(31, 20, 10,'B (30, 20, 10)');
text(-9, -20, -25,'C (-10, -20, -30)');
hold off;
