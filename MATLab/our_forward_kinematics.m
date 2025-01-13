function T = our_forward_kinematics(L1,L4,theta1, theta4)

    %theta1 = theta3
    %theta2 = theta4 = theta_ex

    %L1 = 110;
    %L4 = 160;

    px = L1*cosd(theta1) + L4*cosd(theta4);
    py = L1*sind(theta1) + L4*sind(theta4);

    T = [1, 0, 0, px;
     0, 1, 0, py;
     0, 0, 1, 0;
     0, 0, 0, 1];

end
