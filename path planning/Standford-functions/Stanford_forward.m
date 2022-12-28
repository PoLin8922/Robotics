function [x, y, z, u, v, w] = Stanford_forward(theta1, theta2, d3, theta4, theta5, theta6)
    %% Add the directory containing relevant functions to the path variables
   addpath('./Standford-functions/');

    %% Define the input parameters 
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

    [x, y, z] = Euler_To_Cartesian(Trans);
    % z direction in global frame (for drawing arrow of 3D Cartesian path
    u = Trans(1,3);
    v = Trans(2,3);
    w = Trans(3,3);
end
