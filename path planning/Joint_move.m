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
inch = 2.54;

A = [ 0  0 -1  10/inch
     -1  0  0  20/inch
      0  1  0  30/inch
      0  0  0        1];
  
B = [ 1  0  0 30/inch
      0  1  0 20/inch
      0  0  1 10/inch
      0  0  0       1];
  
C = [ 0 -1  0 -10/inch
      0  0  1 -20/inch
     -1  0  0 -30/inch
      0  0  0        1];
   
%% Find correspomding ，
joint_variable_a = Stanford_inverse(A);
joint_variable_b = Stanford_inverse(B);
joint_variable_c = Stanford_inverse(C);

%% Path planning
% straight line portion (A to B)
t1 = t_a:t_sampling:t_b-t_acc;
size1 = (t_b-t_acc-t_a)/t_sampling +1;
[p1_s1, v1_s1, a1_s1] = Straight_line_portion(joint_variable_a(1), joint_variable_b(1), t_a, t_b, t1-t_a);
[p2_s1, v2_s1, a2_s1] = Straight_line_portion(joint_variable_a(2), joint_variable_b(2), t_a, t_b, t1-t_a);
[p3_s1, v3_s1, a3_s1] = Straight_line_portion(joint_variable_a(3), joint_variable_b(3), t_a, t_b, t1-t_a);
[p4_s1, v4_s1, a4_s1] = Straight_line_portion(joint_variable_a(4), joint_variable_b(4), t_a, t_b, t1-t_a);
[p5_s1, v5_s1, a5_s1] = Straight_line_portion(joint_variable_a(5), joint_variable_b(5), t_a, t_b, t1-t_a);
[p6_s1, v6_s1, a6_s1] = Straight_line_portion(joint_variable_a(6), joint_variable_b(6), t_a, t_b, t1-t_a);
% derive corresponding Cartesian path
x1 = zeros(size1); y1 = zeros(size1); z1 = zeros(size1); phi1 = zeros(size1); theta1 = zeros(size1); psi1 = zeros(size1); 
for i = 1:size1
    [x1(i), y1(i), z1(i), phi1(i), theta1(i), psi1(i)] = Stanford_forward(p1_s1(i), p2_s1(i), p3_s1(i), p4_s1(i), p5_s1(i), p6_s1(i));
end 

% transition portion 
t2 = t_b-t_acc:t_sampling:t_b+t_acc;
size2 = 2*t_acc/t_sampling +1;
[p1_t, v1_t, a1_t] = Transition_portion(p1_s1(size1), joint_variable_b(1), joint_variable_c(1), t_b, t_c, t2-t_b);
[p2_t, v2_t, a2_t] = Transition_portion(p2_s1(size1), joint_variable_b(2), joint_variable_c(2), t_b, t_c, t2-t_b);
[p3_t, v3_t, a3_t] = Transition_portion(p3_s1(size1), joint_variable_b(3), joint_variable_c(3), t_b, t_c, t2-t_b);
[p4_t, v4_t, a4_t] = Transition_portion(p4_s1(size1), joint_variable_b(4), joint_variable_c(4), t_b, t_c, t2-t_b);
[p5_t, v5_t, a5_t] = Transition_portion(p5_s1(size1), joint_variable_b(5), joint_variable_c(5), t_b, t_c, t2-t_b);
[p6_t, v6_t, a6_t] = Transition_portion(p6_s1(size1), joint_variable_b(6), joint_variable_c(6), t_b, t_c, t2-t_b);
% derive corresponding Cartesian path
x2 = zeros(size2); y2 = zeros(size2); z2 = zeros(size2); phi2 = zeros(size2); theta2 = zeros(size2); psi2 = zeros(size2);
for i = 1:size2
   [x2(i), y2(i), z2(i), phi2(i), theta2(i), psi2(i)] = Stanford_forward(p1_t(i), p2_t(i), p3_t(i), p4_t(i), p5_t(i), p6_t(i));
end

% straight line portion (B to C)
t3 = t_b+t_acc:t_sampling:t_c;
size3 = (t_c-t_b-t_acc)/t_sampling +1;
[p1_s2, v1_s2, a1_s2] = Straight_line_portion(joint_variable_b(1), joint_variable_c(1), t_b, t_c, t3-t_b);
[p2_s2, v2_s2, a2_s2] = Straight_line_portion(joint_variable_b(2), joint_variable_c(2), t_b, t_c, t3-t_b);
[p3_s2, v3_s2, a3_s2] = Straight_line_portion(joint_variable_b(3), joint_variable_c(3), t_b, t_c, t3-t_b);
[p4_s2, v4_s2, a4_s2] = Straight_line_portion(joint_variable_b(4), joint_variable_c(4), t_b, t_c, t3-t_b);
[p5_s2, v5_s2, a5_s2] = Straight_line_portion(joint_variable_b(5), joint_variable_c(5), t_b, t_c, t3-t_b);
[p6_s2, v6_s2, a6_s2] = Straight_line_portion(joint_variable_b(6), joint_variable_c(6), t_b, t_c, t3-t_b);
% derive corresponding Cartesian path
x3 = zeros(size3); y3 = zeros(size3); z3 = zeros(size3); phi3 = zeros(size3); theta3 = zeros(size3); psi3 = zeros(size3);
for i = 1:size3
   [x3(i), y3(i), z3(i), phi3(i), theta3(i), psi3(i)] = Stanford_forward(p1_s2(i), p2_s2(i), p3_s2(i), p4_s2(i), p5_s2(i), p6_s2(i));
end

%%  Draw path
% position configuration
figure(1);
subplot(3,2,1); plot(t1,p1_s1,'b', t2,p1_t,'b', t3,p1_s2,'b' ); title('joint1');
subplot(3,2,2); plot(t1,p2_s1,'b', t2,p2_t,'b', t3,p2_s2,'b' ); title('joint2');
subplot(3,2,3); plot(t1,p3_s1*inch,'b', t2,p3_t*inch,'b', t3,p3_s2*inch,'b' ); title('d3'); ylabel('position (degree, cm)')  
subplot(3,2,4); plot(t1,p4_s1,'b', t2,p4_t,'b', t3,p4_s2,'b' ); title('joint4');
subplot(3,2,5); plot(t1,p5_s1,'b', t2,p5_t,'b', t3,p5_s2,'b' ); title('joint5'); xlabel('time (s)')
subplot(3,2,6); plot(t1,p6_s1,'b', t2,p6_t,'b', t3,p6_s2,'b' ); title('joint6'); xlabel('time (s)')

% velocity configuration
figure(2);
subplot(3,2,1); plot(t1,v1_s1,'b', t2,v1_t,'b', t3,v1_s2,'b' ); title('joint1');
subplot(3,2,2); plot(t1,v2_s1,'b', t2,v2_t,'b', t3,v2_s2,'b' ); title('joint2');
subplot(3,2,3); plot(t1,v3_s1*inch,'b', t2,v3_t*inch,'b', t3,v3_s2*inch,'b' ); title('joint3'); ylabel('velocity (degree/s, cm/s)')  
subplot(3,2,4); plot(t1,v4_s1,'b', t2,v4_t,'b', t3,v4_s2,'b' ); title('joint4');
subplot(3,2,5); plot(t1,v5_s1,'b', t2,v5_t,'b', t3,v5_s2,'b' ); title('joint5'); xlabel('time (s)')
subplot(3,2,6); plot(t1,v6_s1,'b', t2,v6_t,'b', t3,v6_s2,'b' ); title('joint6'); xlabel('time (s)')

% acceleration configuration
figure(3);
subplot(3,2,1); plot(t1,a1_s1,'b', t2,a1_t,'b', t3,a1_s2,'b' ); title('joint1');
subplot(3,2,2); plot(t1,a2_s1,'b', t2,a2_t,'b', t3,a2_s2,'b' ); title('joint2');
subplot(3,2,3); plot(t1,a3_s1*inch,'b', t2,a3_t*inch,'b', t3,a3_s2*inch,'b' ); title('joint3'); ylabel('accelation (degree/s^2, cm/s^2)')  
subplot(3,2,4); plot(t1,a4_s1,'b', t2,a4_t,'b', t3,a4_s2,'b' ); title('joint4');
subplot(3,2,5); plot(t1,a5_s1,'b', t2,a5_t,'b', t3,a5_s2,'b' ); title('joint5'); xlabel('time (s)')
subplot(3,2,6); plot(t1,a6_s1,'b', t2,a6_t,'b', t3,a6_s2,'b' ); title('joint6'); xlabel('time (s)')

% Cartesian path
figure(4);
% transform inch to cm
x1 = x1*inch; y1 = y1*inch; z1 = z1*inch; 
x2 = x2*inch; y2 = y2*inch; z2 = z2*inch; 
x3 = x3*inch; y3 = y3*inch; z3 = z3*inch; 
% draw position motion of end-effector 
plot3(x1, y1, z1,'b', x2, y2, z2,'b', x3, y3, z3, 'b');
title('3D Cartesian path'); xlabel('x (cm)');  ylabel('y (cm)');  zlabel('z (cm)');
grid on;
hold on;
% draw orientation motion of end-effector
q1 = quiver3(x1, y1, z1, phi1*5, theta1*5, psi1*5, 0);
q1.Color = 'c';
q1.MaxHeadSize = 0.1;
q2 = quiver3(x2, y2, z2, phi2*5, theta2*5, psi2*5, 0);
q2.Color = 'c';
q2.MaxHeadSize = 0.1;
q3 = quiver3(x3, y3, z3, phi3*5, theta3*5, psi3*5, 0);
q3.Color = 'c';
q3.MaxHeadSize = 0.1;
% draw point A, B and C 
plot3(10, 20, 30,'x', 30, 20, 10,'x', -10, -20, -30, 'x', 'color', 'r');
text(11, 20, 30,'A (10, 20, 30)');
text(31, 20, 10,'B (30, 20, 10)');
text(-9, -20, -25,'C (-10, -20, -30)');
hold off;

    
   