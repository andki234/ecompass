% Close and clear
clear all;
close all;
format long;

% Define the serial port and baud rate
serialPort = 'COM11';
baudRate = 3000000;

% Create a serial port object
device = serialport(serialPort, baudRate);

% Initialize empty arrays to store the data
timestamps_mag = [];
timestamps_gyro_acc = [];
mag_data = [];
gyro_data = [];
acc_data = [];

% File to save data
saveFileName = 'sensor_raw_data.mat';

valid_data = false;

% Pre start Start timer 
tic;

% Read and process data collect 5000 samples
while toc < (60)
    % Check if data is available to read
    if device.NumBytesAvailable > 0
        % Read available data from the serial port
        jsonData = readline(device);
        
        % Decode the JSON data with error checking
        try
            data = jsondecode(jsonData);
            % Start the timer when first ok decoded packet i received
            if (valid_data == false)
                tic;
                valid_data = true;
            end
        catch ME
            %warning('Failed to decode JSON: %s', jsonData);
            continue;
        end

        if isfield(data, 'sampletime') && isfield(data, 'mag_data') && isstruct(data.mag_data)
            % Check if sampletime is a struct and convert accordingly
            if isstruct(data.sampletime)
                % Convert sampletime to table (assuming scalar structure or single row/column structure array)
                if isscalar(data.sampletime) || isvector(data.sampletime)
                    sampletimeTable = struct2table(data.sampletime);
                else
                    error('sampletime must be a scalar structure or a single row/column structure array.');
                end
            else
                % If sampletime is not a struct, convert it to a table directly
                sampletimeTable = array2table(data.sampletime, 'VariableNames', {'SampleTime'});
            end
    
            % Convert mag_data to table
            magDataTable = struct2table(data.mag_data);
            
            % Concatenate sampletimeTable and magDataTable horizontally
            newRow = [sampletimeTable, magDataTable];
            
            % Append the new row to the existing mag_data table
            mag_data = [mag_data; newRow];
        end

        if isfield(data, 'sampletime') && isfield(data, 'acc_data') && isstruct(data.acc_data)
            % Check if sampletime is a struct and convert accordingly
            if isstruct(data.sampletime)
                % Convert sampletime to table (assuming scalar structure or single row/column structure array)
                if isscalar(data.sampletime) || isvector(data.sampletime)
                    sampletimeTable = struct2table(data.sampletime);
                else
                    error('sampletime must be a scalar structure or a single row/column structure array.');
                end
            else
                % If sampletime is not a struct, convert it to a table directly
                sampletimeTable = array2table(data.sampletime, 'VariableNames', {'SampleTime'});
            end
    
            % Convert acc_data to table
            accDataTable = struct2table(data.acc_data);
            
            % Concatenate sampletimeTable and accDataTable horizontally
            newRow = [sampletimeTable, accDataTable];
            
            % Append the new row to the existing acc_data table
            acc_data = [acc_data; newRow];
        end

        if isfield(data, 'sampletime') && isfield(data, 'gyro_data') && isstruct(data.gyro_data)
            % Check if sampletime is a struct and convert accordingly
            if isstruct(data.sampletime)
                % Convert sampletime to table (assuming scalar structure or single row/column structure array)
                if isscalar(data.sampletime) || isvector(data.sampletime)
                    sampletimeTable = struct2table(data.sampletime);
                else
                    error('sampletime must be a scalar structure or a single row/column structure array.');
                end
            else
                % If sampletime is not a struct, convert it to a table directly
                sampletimeTable = array2table(data.sampletime, 'VariableNames', {'SampleTime'});
            end
    
            % Convert gyro_data to table
            gyroDataTable = struct2table(data.gyro_data);
            
            % Concatenate sampletimeTable and gyroDataTable horizontally
            newRow = [sampletimeTable, gyroDataTable];
            
            % Append the new row to the existing gyro_data table
            gyro_data = [gyro_data; newRow];
        end
    end
end

% Cut away first two samples for sensor data 
mag_data = mag_data(3:end, :);
gyro_data = gyro_data(3:end, :);
acc_data = acc_data(3:end, :);

% Save the final data
save(saveFileName, 'mag_data', 'gyro_data', 'acc_data');

% Clear all variables except the data
clearvars -except mag_data gyro_data acc_data;

