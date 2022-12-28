function [] = Inverse_Outout(theta_1, theta_2, d3, theta_4, theta_5, theta_6)

    disp('Corresponding variables (θ1 θ2 d3 θ4 θ5 θ6) :');
    
    if theta_1<-160 || theta_1>160 
        disp('θ1 is out of range!');
    end
    
    if theta_2<-125 || theta_2>125
        disp('θ2 is out of range!');
    end
    
    if d3<-30 || d3>30
        disp('d3 is out of range!');
    end
    
    if theta_4<-140 || theta_4>140
        disp('θ4 is out of range!');
    end
    
    if theta_5<-100 || theta_5>100
        disp('θ5 is out of range!');
    end
    
    if theta_6<-260 || theta_6>260
        disp('θ6 is out of range!');
    end
    fprintf("%.6f %.6f %.6f %.6f %.6f %.6f\n\n", theta_1, theta_2, d3, theta_4, theta_5, theta_6);