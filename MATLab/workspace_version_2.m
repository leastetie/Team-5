close all;
clear all;

% Declare global variables
global hPlot hFig x y;

% Create GUI
hFig = figure;

% Create plot area
hPlot = axes('Position', [0.2, 0.35, 0.6, 0.6]);

% Initialize variables for plotting
x = [];
y = [];

% Define robot link lengths 
L1 = 110;
L2 = 60;
L3 = 110; 
L4 = 160;
L5 = 0; 
L_ex = 100; 

% Define the range and resolution for joint angles
theta1_range = linspace(0, 180, 50);
theta4_range = linspace(0, 180, 50); 

% Iterate through the specified ranges of joint angles
for theta1 = theta1_range
    for theta4 = theta4_range
                % Compute end-effector positions using forward kinematics
                [x,y] = forward_kinematics_version_2(theta1, theta4);
                
                % Store the positions
                x = [x];
                y = [y];


     end

end

% Plot the workspace
scatter(hPlot, x, y, 20, 'filled');
xlabel('X Position');
ylabel('Y Position');
title('Workspace of Parallel SCARA Robot');
axis equal;
grid on;

% Plot the square of 156x156 mm area starting from y = 110 (offset)
x_square_start = -80;   % x starting position 
y_square_start = 110; % y starting position
square_size = 156;    % Size of the square

% Draw the square
rectangle('Position', [x_square_start, y_square_start, square_size, square_size], ...
          'EdgeColor', 'r', 'LineWidth', 2);

