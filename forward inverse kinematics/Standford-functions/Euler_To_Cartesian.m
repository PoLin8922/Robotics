function [x, y, z, A, B, C] = Euler_To_Cartesian(Trans)
    x = Trans(1,4);
    y = Trans(2,4);
    z = Trans(3,4);
    %euler angle z-y-z (ϕ, θ, ψ)
    a = atan2(Trans(2, 3), Trans(1, 3));
    A = a*180/pi;
    B = (atan2((Trans(1, 3)*cos(a)) + (Trans(2, 3)*sin(a)), Trans(3,3))*180/pi);
    C = (atan2((-1*Trans(1, 1)*sin(a) + (Trans(2, 1)*cos(a))), (-1*Trans(1, 2)*sin(a)) + (Trans(2, 2)*cos(a)))*180/pi);