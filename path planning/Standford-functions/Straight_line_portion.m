function [position, velocity, acceleration] = Straight_line_portion(point_a, point_b, t_a, t_b, t)
    % define parameters
    t_sampling = 0.002;
    t_acc = 0.2;
    delta_t = t_b - t_a;
    delta_b = point_b - point_a;
    size = int32((t_b-t_acc-t_a)/t_sampling) + 1; %dimention for velocity and acceleration
    
    % calculate position, velocity, accleration at different time h
    h = t/delta_t;
    position = delta_b.*h + point_a;
    velocity = zeros(size); velocity(:) = delta_b/delta_t;
    acceleration = zeros(size); acceleration(:) = 0;
end
