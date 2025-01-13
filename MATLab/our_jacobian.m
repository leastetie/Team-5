function J = our_jacobian(r1,r2,t1,t2)
    N = 2;  % Number of joints
    J = zeros(2, N);  % Initialize the Jacobian matrix
    
    J = [-r1*sind(t1), -r2*sind(t2);
        r1*cosd(t1), r2*cosd(t2)];
end