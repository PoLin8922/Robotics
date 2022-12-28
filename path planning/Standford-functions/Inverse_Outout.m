function index = Inverse_Outout(theta_1, theta_2, d3, theta_4, theta_5, theta_6, i, i_last)
    % judging whether joint variables exceeding the working space
    % if exceeding working space flag equal to 0 
    
    flag = 1;
    if theta_1<-160 || theta_1>160 
        flag = 0;
    end
    
    if theta_2<-125 || theta_2>125
        flag = 0;
    end
    
    if d3<-30 || d3>30
        flag = 0;
    end
    
    if theta_4<-140 || theta_4>140
        flag = 0;
    end
    
    if theta_5<-100 || theta_5>100
        flag = 0;
    end
    
    if theta_6<-260 || theta_6>260
        flag = 0;
    end

    index = i_last;
    if flag==1
        index = i;
    end
end
        