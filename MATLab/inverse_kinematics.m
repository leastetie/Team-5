function [theta1, theta4] = inverse_kinematics(x, y)
    L1 = 110;
    L2 = 60;
    %L3 = 110; %same as L1
    L4 = 160;

    r = sqrt(x^2 + y^2);
    A = acosd((L1^2 + r^2 - L4^2) / (2*L1*r));
    B = acosd((L1^2 + L4^2 - r^2) / (2*L1*L4));
    %C = (r^2 + L4^2 - L1^2) / (2*r*L4); %not used
    if x>=0 && y>=0
        D = atand(y/x);
    elseif x<0 && y>=0
        D = 180 - atand(y/-x);
    elseif x<0 && y<0
        error('not in range of motion')
    elseif x>=0 && y<0
        D = atand(y/x);
    else
        error('unforseen situation');
    end
    
    E = 180 - A - D;
    F = B - E;
    W = (L4 - L2) * cosd(F);
    Z = (L4 - L2) * sind(F);
    x2 = x - W;
    y2 = y - Z; % might throw an error when end effector is below (x2, y2)
    
    r2 = sqrt(x2^2 + y2^2);
    G = acosd((r^2 + r2^2 - (L4 - L2)^2) / (2*r*r2));
    H = acosd((L2^2 + r2^2 - L1^2) / (2*L2*r2));

    theta1 = D + A;
    %theta2 = F;
    %theta3= theta1;
    theta4 = (D + G) - H;


end