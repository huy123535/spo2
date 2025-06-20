% MATLAB script to plot AC/DC ratios from MAX30102 sensor
% Connect Arduino to COM port and run this script

% Clear workspace and close all figures
clear all;
close all;

% Create serial port object
s = serialport("COM5", 115200); % Change COM3 to your Arduino port
configureTerminator(s, "LF");

% Initialize arrays for storing data
max_points = 1000; % Maximum number of points to display
time_data = zeros(1, max_points);
red_data = zeros(1, max_points);
ir_data = zeros(1, max_points);
point_count = 0;

% Create figure for plotting
figure('Name', 'AC/DC Ratio Plot', 'NumberTitle', 'off');
h_red = plot(NaN, NaN, 'r-', 'LineWidth', 1.5);
hold on;
h_ir = plot(NaN, NaN, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('AC/DC Ratio');
title('MAX30102 AC/DC Ratio');
legend('Red Signal', 'IR Signal');

% Start time
start_time = tic;

try
    while true
        % Read data from serial port
        if s.NumBytesAvailable > 0
            data = readline(s);
            
            % Parse the data
            if contains(data, 'AC/DC_RED:') && contains(data, 'AC/DC_IR:')
                % Extract values
                parts = split(data, ',');
                red_str = split(parts(1), ':');
                ir_str = split(parts(2), ':');
                
                red_val = str2double(red_str(2));
                ir_val = str2double(ir_str(2));
                
                % Update point count
                point_count = point_count + 1;
                if point_count > max_points
                    % Shift data left
                    time_data = [time_data(2:end), toc(start_time)];
                    red_data = [red_data(2:end), red_val];
                    ir_data = [ir_data(2:end), ir_val];
                else
                    % Append new data
                    time_data(point_count) = toc(start_time);
                    red_data(point_count) = red_val;
                    ir_data(point_count) = ir_val;
                end
                
                % Update plot
                set(h_red, 'XData', time_data(1:point_count), 'YData', red_data(1:point_count));
                set(h_ir, 'XData', time_data(1:point_count), 'YData', ir_data(1:point_count));
                
                % Adjust axis limits
                xlim([max(0, time_data(point_count)-10), time_data(point_count)]);
                ylim([min(min(red_data(1:point_count)), min(ir_data(1:point_count)))*0.9, ...
                      max(max(red_data(1:point_count)), max(ir_data(1:point_count)))*1.1]);
                
                % Force draw
                drawnow;
            end
        end
    end
catch ME
    % Clean up on error or when user stops the script
    clear s;
    rethrow(ME);
end

% Clean up
clear s; 