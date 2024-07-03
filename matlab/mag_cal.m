clear all
close all
format long

% Filename
file_name = 'sensor_raw_data.mat';
file_name_sens = 'sensor_raw_sensivity_data.mat';

% Load vectors from MAT file
load(file_name);

% Use LIS3MDL magnetometer sensitivity (LSB/gauss) for ±4 gauss 
mag_sensitivity = 6842;
mag_data.x = mag_data.x / mag_sensitivity;
mag_data.y = mag_data.y / mag_sensitivity;
mag_data.z = mag_data.z / mag_sensitivity;

% Use LIS6DS3 gyroscope sensitivity (LSB/dps) for ±125 dps
gyro_sensitivity = 114;
gyro_data.x = gyro_data.x / gyro_sensitivity;
gyro_data.y = gyro_data.y / gyro_sensitivity;
gyro_data.z = gyro_data.z / gyro_sensitivity;

% Use LIS6DSOX accelerometer sensitivity (LSB/g) for ±2 g
acc_sensitivity = 16384;
acc_data.x = acc_data.x / acc_sensitivity;
acc_data.y = acc_data.y / acc_sensitivity;
acc_data.z = acc_data.z / acc_sensitivity;

% Save vectors to a MAT file with sensitivity data
save(file_name_sens, 'mag_data', 'gyro_data', 'acc_data');

% Generate mag timestap vector
mag_data_timestamps = mag_data.SampleTime*10^-6;
cumulative_timestamps = cumsum(mag_data_timestamps);
mag_timestamps = [0; cumulative_timestamps(1:end-1)];

% Use the magcal function to determine calibration parameters that correct noisy magnetometer data. 
% The magcal function requires a matrix of magnetometer data with each column representing a different axis.
% The magnetometer data should be collected when the sensor is stationary and rotated to cover all orientations.
% The magcal function returns the calibration parameters and the calibrated magnetometer data.
% The calibration parameters include the offset and scale factor for each axis, as well as the transformation matrix.
% The transformation matrix can be used to convert the calibrated magnetometer data to a different coordinate system.
% The calibrated magnetometer data can be used to improve the accuracy of heading calculations or other magnetic field measurements.
% The calibration parameters can be saved and applied to future magnetometer data to correct for sensor errors.
% The magcal function is part of the Sensor Fusion and Tracking Toolbox.

%  Combine magnetometer data into a matrix
mag_data_matrix = [mag_data.x(:), mag_data.y(:), mag_data.z(:)];

% Perform magnetometer calibration
[A, b, expmfs] = magcal(mag_data_matrix);

% Save calibration matrixes
save('mag_cal_data.mat', 'A','b',"expmfs")

% Apply the calibration transformation
calibrated_mag_data = (mag_data_matrix - b) * A;

% Ensure the timestamps vector matches the length of the calibrated data
calibrated_mag_timestamps = mag_timestamps(1:size(calibrated_mag_data, 1));

% Save calibrated magnetometer data and heading
save('calibrated_sensor_data.mat', 'calibrated_mag_data')

% 3D scatter plot of raw mag data
figure
scatter3(mag_data_matrix(:, 1), mag_data_matrix(:, 2), mag_data_matrix(:, 3), 10, mag_timestamps, 'filled');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Raw Magnetometer Data');
colorbar;

% 3D scatter plot of calibrated mag data
figure
scatter3(calibrated_mag_data(:, 1), calibrated_mag_data(:, 2), calibrated_mag_data(:, 3), 10, calibrated_mag_timestamps, 'filled');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Calibrated Magnetometer Data');
colorbar;

% 3D scatter plot of raw mag data and calibrated mag data
figure
scatter3(mag_data_matrix(:, 1), mag_data_matrix(:, 2), mag_data_matrix(:, 3), 10, 'r', 'filled');
hold on
scatter3(calibrated_mag_data(:, 1), calibrated_mag_data(:, 2), calibrated_mag_data(:, 3), 10, 'g', 'filled');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Raw and Calibrated Magnetometer Data');
legend('Raw Data', 'Calibrated Data');
hold off



