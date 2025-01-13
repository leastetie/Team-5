close all
clear
clc

% Parameters for robot arm
r1 = 1.1;    % Length of first link
r2 = 1.6;    % Length of second link
r3 = 0.6;    % Length of third link
t1 = 10;   % Initial angle of joint 1
t2 = -10;      % Initial angle of joint 2

% Desired position
x_d = [-0.5; 1]; % Desired x and y coordinates of the end-effector

% Control parameters
damping_factor = 2;   % Damping factor for DLS
step_limit = 1;        % Maximum step size for target clamping
gain = 2;              % Gain for Jacobian transpose method
tolerance = 1e-3;        % Position error tolerance
max_iterations = 1000;   % Maximum number of iterations
dt = 0.01;               % Time step for plotting manipulability
ellipsoid_scale = 0.25;   % Scaling factor for ellipsoids

% Initialize array to store manipulability values
manipulability_values = zeros(max_iterations, 1);

figure;
hold on;
axis equal;
xlim([-2, 2]);
ylim([-2, 2]);
xlabel('X');
ylabel('Y');
title('Robot Arm Motion and Manipulability Ellipsoids');

for i = 1:max_iterations
    % Calculate forward kinematics to get the current end-effector position
    T = our_forward_kinematics(r1, r2, t1, t2);
    x = T(1, 4);
    y = T(2, 4);
    current_position = [x; y];
    
    % Calculate position error between desired and current end-effector position
    error = x_d - current_position;
    
    % Target clamping: Limit the step size to avoid large jumps
    if norm(error) > step_limit
        error = step_limit * (error / norm(error));
    end
    
    % Compute Jacobian for the current joint angles
    J = our_jacobian(r1, r2, t1, t2);
    
    % Calculate manipulability as the square root of the determinant of (J*J')
    manipulability =sqrt(det(J*J'));
    manipulability_values(i) = manipulability;
    
    % Damped Least Squares (DLS) method for stable inverse calculation
    inv_J_damped = (J' * J * damping_factor^2 * eye(size(3,2))) \ J';
    delta_q_dls = inv_J_damped * error;
    
    % Jacobian Transpose method as an alternative to inverse Jacobian
    delta_q_transpose = gain * J' * error;
    
    % Select method: uncomment one of the two lines below
  delta_q = delta_q_dls;     % Damped Least Squares method
  % delta_q = delta_q_transpose; % Jacobian Transpose method
    
    % Update joint angles using calculated change
    t1 = t1 + delta_q(1);
    t2 = t2 + delta_q(2);
    
    % Plot current robot arm position
    plot([0, r1*cosd(t1), r1*cosd(t1) + r2*cosd(t2)], ...
         [0, r1*sind(t1), r1*sind(t1) + r2*sind(t2)], ...
         [0, r3*cosd(t2), r3*cosd(t2)+ r1*cosd(t1)], ...
      [0, r3*sind(t2), r3*sind(t2) + r1*sind(t1)], '-o');
    
    % Plot the manipulability ellipsoid at the current configuration
    [U, S, V] = svd(J);  % Singular Value Decomposition of the Jacobian
    theta = linspace(0, 2*pi, 100);
    ellipse_x = ellipsoid_scale * S(1,1) * cos(theta);%.....apply scaling
    ellipse_y = ellipsoid_scale * S(2,2) * sin(theta);%.....apply scaling  
    ellipse = [ellipse_x; ellipse_y]; %Rotate ellipse based on current Jacobian ...
    
    % Plot the ellipsoid around the current end-effector position
    plot(ellipse(1, :) + x, ellipse(2, :) + y, 'm');
    
    pause(0.01);  % Pause for visualization
    
    % Check if the end-effector is within the desired tolerance
    if norm(error) < tolerance
        disp('Desired position reached!');
        break;
    end
end

% Plot manipulability over the trajectory
figure;
plot(manipulability_values(1:i));
xlabel('Iteration');
ylabel('Manipulability');
title('Manipulability across the trajectory');
