function joint_variable = Stanford_inverse(Trans)
    %% Add the directory containing relevant functions to the path variables
    addpath('./Standford-functions/');

    %% Define the parameters 
    d2 = 6.375;

    %% inverse kinamic
    %θ1, 2 solutions
    theta_1 = zeros(1,8);
    theta_1(1:4) = (atan2(Trans(2, 4), Trans(1, 4)) - atan2(d2, sqrt(Trans(1, 4)^2 + Trans(2, 4)^2 - d2^2)))*180/pi;
    theta_1(5:8) = (atan2(Trans(2, 4), Trans(1, 4)) - atan2(d2, -sqrt(Trans(1, 4)^2 + Trans(2, 4)^2 - d2^2)))*180/pi;
    for i = 1:8
        if theta_1(i) < -180 
            theta_1(i) = theta_1(i)+360; % keep theta1 between 0~180 or 0~-180
        end
    end


    %θ2, 4 solutions
    theta_2 = zeros(1,8);
    %for θ1 is positive , 2 solutions
    theta_2(1:2) = atan2(cosd(theta_1(1))*Trans(1,4) + sind(theta_1(1))*Trans(2,4),Trans(3,4))*180/pi;
    theta_2(3:4) = atan2(-cosd(theta_1(1))*Trans(1,4) - sind(theta_1(1))*Trans(2,4),-Trans(3,4))*180/pi;
    %for θ1 is negative , 2 solutions
    theta_2(5:6) = atan2(cosd(theta_1(5))*Trans(1,4) + sind(theta_1(5))*Trans(2,4),Trans(3,4))*180/pi;
    theta_2(7:8) = atan2(-cosd(theta_1(5))*Trans(1,4) - sind(theta_1(5))*Trans(2,4),-Trans(3,4))*180/pi;

    %d3 , 8 solutions
    d3 = zeros(1,8);
    for i = 1:8
        d3(i) = Trans(3,4)/cosd(theta_2(i));  
    end

    %θ4 , 8 solutions
    theta_4 = zeros(1,8);
    for i = 1:2:8
        %numerator
        num = -sind(theta_1(i))*Trans(1,3) + cosd(theta_1(i))*Trans(2,3);
        %denominator
        den = cosd(theta_1(i))*cosd(theta_2(i))*Trans(1,3) + sind(theta_1(i))*cosd(theta_2(i))*Trans(2,3) - sind(theta_2(i))*Trans(3,3);
        % if sin(θ5) > 0
        theta_4(i) = atan2(num, den)*180/pi;
        % if sin(θ5) < 0
        theta_4(i+1) = atan2(-num, -den)*180/pi; 
    end

    %θ5 , 8 solutions
    theta_5 = zeros(1,8);
    for i = 1:2:8
        %numerator
        num = cosd(theta_4(i))*(cosd(theta_1(i))*cosd(theta_2(i))*Trans(1,3) + sind(theta_1(i))*cosd(theta_2(i))*Trans(2,3) - sind(theta_2(i))*Trans(3,3)) + sind(theta_4(i))*(-sind(theta_1(i))*Trans(1,3) + cosd(theta_1(i))*Trans(2,3));
        %denominator
        den = cosd(theta_1(i))*sind(theta_2(i))*Trans(1,3) + sind(theta_1(i))*sind(theta_2(i))*Trans(2,3) + cosd(theta_2(i))*Trans(3,3);
        % make sin(θ5) > 0
        theta_5(i) = atan2(num, den)*180/pi;
        % make sin(θ5) < 0
        theta_5(i+1) = atan2(-num, den)*180/pi; 
    end

    %θ6 , 8 solutions
    theta_6 = zeros(1,8);
    for i = 1:2:8
        %numerator
        num = cosd(theta_1(i))*sind(theta_2(i))*Trans(1,2) + sind(theta_1(i))*sind(theta_2(i))*Trans(2,2) + cosd(theta_2(i))*Trans(3,2);
        %denominator
        den = -cosd(theta_1(i))*sind(theta_2(i))*Trans(1,1) - sind(theta_1(i))*sind(theta_2(i))*Trans(2,1) - cosd(theta_2(i))*Trans(3,1);
        % if sin(θ5) > 0
        theta_6(i) = atan2(num, den)*180/pi;  
        % if sin(θ5) < 0
        theta_6(i+1) = atan2(-num, -den)*180/pi;  
    end


    %%  output
    % choose the first set of joint variables that not out of the working space
    index = 8;
    for i = 8:-1:1
        index = Inverse_Outout(theta_1(i), theta_2(i), d3(i), theta_4(i), theta_5(i), theta_6(i), i, index);
    end
    
    joint_variable = [theta_1(index), theta_2(index), d3(index), theta_4(index), theta_5(index), theta_6(index)];

end