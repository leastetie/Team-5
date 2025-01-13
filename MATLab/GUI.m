close all
clear

% declare global variables
%   s           serial port communication
%   hInput1     input widget 1
%   hInput1     input widget 2
%   hPlot       plot widget
%   hFig        figure widget
%   hTimer      continuous timer
%   c           command type
%   y1          data stream series 1
%   y2          data stream series 2
global s hInputX hInputY hPlot hFig hTimer desiredX desiredY actualX actualY c

%% Set up
% Create serial port object
s = serialport("COM6", 115200);
configureTerminator(s,"CR/LF");
s.UserData = struct("Data",[],"Count",1);
clc
% Create GUI
hFig = figure('Name', 'Arduino Communication Interface', 'NumberTitle', 'off','CloseRequestFcn', @closeGUI);

% Create input field for sending commands to microcontroller
uicontrol('Style', 'text', 'Position', [20,50,100,20], 'String', 'Input 1:');
hInputX = uicontrol('Style', 'edit', 'Position', [120, 50, 100, 25]);

uicontrol('Style', 'text', 'Position', [20,20,100,20], 'String', 'Input 2:');
hInputY = uicontrol('Style', 'edit', 'Position', [120, 20, 100, 25]);

% Create button for sending commands
hSend = uicontrol('Style', 'pushbutton', 'String', 'Send', 'Position', [240, 35, 100, 25], 'Callback', @sendCommand);

% Create plot area
hPlot = axes('Position', [0.2, 0.35, 0.6, 0.6]);
xlabel(hPlot, 'X Position');
ylabel(hPlot, 'Y Position');
title(hPlot, 'End-Effector Position');
grid(hPlot, 'on');
hold(hPlot, 'on');

% Initialize variables for plotting
c = [];
desiredX = [];
desiredY = [];
actualX = [];
actualY = [];

% Set up timer for continuously receiving data from Arduino
hTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.1, 'TimerFcn', @readDataTimer);
start(hTimer);


%% Callback function for sending commands
function sendCommand(~, ~)
    global s hInputX hInputY desiredX desiredY;

    % Get values from input fields
    x = str2double(get(hInputX, 'String'));
    y = str2double(get(hInputY, 'String'));


    % Compute inverse kinematics to get joint angles
    [theta1, theta4] = inverse_kinematics(x, y);
    %

    % Format command string
    cmdStr = sprintf("C%.2f,%.2f;", theta1, theta4);

    % Send command string to Arduino
    try
        write(s, cmdStr, "string");
        % Store desired positions for plotting
        desiredX = [desiredX, x];
        desiredY = [desiredY, y];
    catch
        errordlg('Failed to send data to Arduino. Check the connection.', 'Communication Error');
    end
end

%% Callback function for reading data from Arduino
function readDataTimer(~, ~)
    global s hPlot actualX actualY desiredX desiredY c;

    % Read data from serial port
    if s.NumBytesAvailable > 0
        dataStr = readline(s);
        % Parse data values from string
        data = sscanf(dataStr, 'c%f,%f,%f');
        if numel(data) == 3
            theta1 = data(2);
            theta4 = data(3);
            % Compute forward kinematics to get actual positions
            [x, y] = forward_kinematics_version_2(theta1, theta4);
            % Store actual positions for plotting
            actualX = [actualX, x];
            actualY = [actualY, y];
            % Update plot
            plot(hPlot, actualX, actualY, 'b.-');
            plot(hPlot, desiredX, desiredY, 'r.-');
            drawnow;
        end
    end
end

%% Callback function for closing the GUI
function closeGUI(~, ~)
    global s hTimer;

    % Stop and delete timer
    stop(hTimer);
    delete(hTimer);

    % Close serial port
    if isvalid(s)
        clear s;
    end

    % Close GUI
    delete(gcf);
end



