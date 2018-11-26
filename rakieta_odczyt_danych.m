
%magx %magy %magz

filename_sensors = './logi_rakieta/Stachurski_1_sensor_combined_0.csv';
filename_mag = './logi_rakieta/Stachurski_1_vehicle_magnetometer_0.csv';

D_sensors = csvread(filename_sensors, 1);
D_mag = csvread(filename_mag, 1);

time_acc_gyro = D_sensors(:, 1);
acc =  D_sensors(:, 7:10); %columns in order: x, y, z, dt [us]
gyro = D_sensors(:, 2:5);%columns in order: x, y, z, dt [us]
compass = D_mag(:, 1:4);%columns in order: timestamp [us], x, y, z
time_compass = D_mag(:, 1);

figure();
plot(time_acc_gyro(:, 1)/1e6, acc(:, 1));
hold on;
plot(time_acc_gyro(:, 1)/1e6, acc(:, 2));
hold on;
plot(time_acc_gyro(:, 1)/1e6, acc(:, 3));
hold on;
legend('x', 'y', 'z');
xlabel('czas [s]');
grid on;
title('akcelerometr');

figure();
plot(time_acc_gyro(:, 1)/1e6, gyro(:, 1));
hold on;
plot(time_acc_gyro(:, 1)/1e6, gyro(:, 2));
hold on;
plot(time_acc_gyro(:, 1)/1e6, gyro(:, 3));
legend('x', 'y', 'z');
xlabel('czas [s]');
grid on;
title('zyroskop');

figure();
plot(time_compass(:, 1)/1e6, compass(:, 2));
hold on;
plot(time_compass(:, 1)/1e6, compass(:, 3));
hold on;
plot(time_compass(:, 1)/1e6, compass(:, 4));
legend('x', 'y', 'z');
xlabel('czas [s]');
grid on;
title('magnetometr');

%gyr_x, gyr_y, gyr_z
%acc_x, acc_y, acc_z

%timestamp,gyro_rad[0],gyro_rad[1],gyro_rad[2],gyro_integral_dt,
%accelerometer_timestamp_relative,accelerometer_m_s2[0],accelerometer_m_s2[1],
%accelerometer_m_s2[2],accelerometer_integral_dt


