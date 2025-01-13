function [x,y] = forward_kinematics_version_2(theta1, theta4)

    L1 = 110;
    L2 = 60;
    L3 = 110;
    L4 = 60;
    L_ex = 100;

    x1=L1*cosd(theta1)+L2*cosd(theta4)+L_ex*cosd(theta4);
    x2=L4*cosd(theta4)+L3*cosd(theta1)+L_ex*cosd(theta4);

    y1=L1*sind(theta1)+L2*sind(theta4)+L_ex*sind(theta4);
    y2=L4*sind(theta4)+L3*sind(theta1)+L_ex*sind(theta4);

    x = x1;
    y = y1;


end

