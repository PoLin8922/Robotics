function [x, y, z, phi, theta, psi] = Euler_To_Cartesian(Trans)
    x = Trans(1,4);
    y = Trans(2,4);
    z = Trans(3,4);
    
    % euler angles rotation, z-y-z (ϕ, θ, ψ)
    phi = atan2(Trans(2, 3), Trans(1, 3))*180/pi;
    theta = atan2((Trans(1, 3)*cos(phi)) + (Trans(2, 3)*sin(phi)), Trans(3,3))*180/pi;
    psi = atan2((-1*Trans(1, 1)*sin(phi) + (Trans(2, 1)*cos(phi))), (-1*Trans(1, 2)*sin(phi)) + (Trans(2, 2)*cos(phi)))*180/pi;
end