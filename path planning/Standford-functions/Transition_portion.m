function [position, velocity, acceleration] = Transition_portion(point_a_p, point_b, point_c, t_b, t_c, t)
    % define parameters
    t_acc = 0.2;
    T = t_c - t_b;
    delta_b = point_a_p - point_b;
    delta_c = point_c - point_b;
    
    % calculate position, velocity, accleration at different time h
    h = (t+t_acc)/(2*t_acc);
    position = ((delta_c.*t_acc/T + delta_b).*(2-h).*(h.^2) - 2*delta_b).*h + point_b + delta_b;
    velocity = ((delta_c.*t_acc/T + delta_b).*(1.5-h).*2.*(h.^2) - delta_b)/t_acc;
    acceleration = ((delta_c.*t_acc/T + delta_b)*(1-h))*3.*h/t_acc/t_acc;
end