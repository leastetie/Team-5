x=100;
y=100;

[theta1, theta4] = inverse_kinematics(x, y);
[x, y] = forward_kinematics_version_2(theta1, theta4);

fprintf('Desired position: x = %.2f, y = %.2f\n', x, y);
fprintf('Calculated thetas: theta1 = %.2f degrees, theta4 = %.2f degrees\n', theta1, theta4);
fprintf('Calculated positions (verification): x = %.2f, y = %.2f\n', x, y);