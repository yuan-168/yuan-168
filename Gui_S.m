% Communicaiton between Arduino and MATLAB
%   @author         Alejandro Granados
%   @organisation   King's College London
%   @module         Medical Robotics Hardware Development
%   @year           2023

%close all
clear all

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
global s hInput1 hInput2 hPlot hFig hTimer c y1 y2 r1 r2 t1 t2

%% Set up
% Create serial port object
s = serialport('/dev/cu.usbmodem1101', 115200);
configureTerminator(s,"CR/LF");
s.UserData = struct("Data",[],"Count",1)

% Create GUI
hFig = figure;
% Define the new coordinate range
xMin = -40;
xMax = 180;
yMin = -40;
yMax = 180;

% Create input field for sending commands to microcontroller
hInput1 = uicontrol('Style', 'edit', 'Position', [20, 20, 100, 25]);
hInput2 = uicontrol('Style', 'edit', 'Position', [120, 20, 100, 25]);

% Create button for sending commands
hSend = uicontrol('Style', 'pushbutton', 'String', 'Send', 'Position', [20, 50, 100, 25], 'Callback', @sendCommand);

% Create plot area
hPlot = axes('Position', [0.2, 0.2, 0.6, 0.6],'XLim', [xMin, xMax], 'YLim', [yMin, yMax]);

% Set up variables for real-time plotting
c = [];
y1 = [];
y2 = [];
t0 = now;
r1 = 108;
r2 = 110;
% Set up timer for continuously receiving data from microcontroller
hTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.05, 'TimerFcn', @readDataTimer);
start(hTimer);
hFig.CloseRequestFcn = @closeGUI;



%% Callback function for sending commands
function sendCommand( ~, ~)
global s hInput1 hInput2
    % Get values from input fields as strings but convert to numbers
    x = str2num(get(hInput1, 'String'));
    y = str2num(get(hInput2, 'String'));
    px = x+42;
    py = y-78;
    % validate input
    if numel(px) ~= 1
        return; % input field must contain one value
    end
    if numel(py) ~= 1
        return; % input field must contain one value
    end 
     Ik = ik_differential(0, 0, px, py);
    % Extract the first target point
     target1 = Ik(:, 2);
     [t1,t2] = inverse_kinematics(108,110,target1(1),target1(2)) 
     % Format command values as a string, e.g., C40.0,3.5;
     cmdStr = sprintf("C%.2f,%.2f;", t1, t2);
     % Send command string to microcontroller
     write(s, cmdStr, "string");

for i = 2:size(Ik, 2)-1
     px = Ik(1, i);
     py = Ik(2, i);
     
    % Check the distance between the current position and the first target point
    % Set a threshold for when to send the second set of joint angles
    if norm([px; py] - target1) < 0.3
        pause(0.1);
        % Get the second set of joint angles
        [t1, t2] = inverse_kinematics(108, 110, px, py);
        
        % Format command values as a string, e.g., C40.0,3.5;
        cmdStr = sprintf("C%.2f,%.2f;", t1, t2);

        % Send command string to microcontroller
        write(s, cmdStr, "string");
        disp(cmdStr);

        % Update the target point for the next iteration
        target1 = Ik(:, i + 1);

        % Increment the count
        s.UserData.Count = s.UserData.Count + 1;
    
      end
   end
end

%% Callback function fo reading time series values from microcontroller
function readDataTimer(~, ~)
global s hPlot c y1 y2 r1 r2 

    % Read the ASCII data from the serialport object.
    dataStr = readline(s);
    if isempty(dataStr)
        pause(0.01);
    else
    % Parse data values from string and add accumulate into arrays
    
    % e.g. Arduino sending 2 series: c1,100
    
    data = sscanf(dataStr, "%c%f,%f");
    c = [c, data(1)];
    a1 = data(2);
    a2 = data(3);
    T = forward_kinematics(108,110,a1,a2);
    x1 = T(1,4);
    x2 = T(2,4);
    z1 = x1 - 42;
    z2 = x2 + 78;
    y1 = [y1, z1]; 
    y2 = [y2, z2];
    % Plot every point
    plot(hPlot, y1, y2, 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
       
    % configure callback 
    % configureCallback(s, "off");
    % clear data;
    % Downsampling - plot every nth data point
    %    downsampling_factor = 50;  % Adjust as needed
    %    if mod(length(y1), downsampling_factor) == 0
    % Plot only every nth data point
           
    % plot(hPlot, y1(end-downsampling_factor+1:end), y2(end-downsampling_factor+1:end), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
    grid on
    % Adjust tick values and labels to match the new coordinate range
  
   
     % Measure frequency
   % toc

    % Reset timer
   % tic;
    end
end
%% Callback function for closing the GUI
function closeGUI(~, ~)
global s hFig hTimer

    % Stop timer
    stop(hTimer);
    delete(hTimer);
    
    % Close serial port
    %delete(s);
    
    % Close GUI
    delete(hFig);
end