% Declare variables
acc = [0,0,0];
gyro = [0,0,0];
mag = [0,0,0];

acc_cal = [0,0,0];
gyro_cal = [0,0,0];

counter = 0;
IMU_available = 0;
IMU_error = 0;
acc_cali = [0,0,0];
gyro_cali = [0,0,0];

% Set circular buffer variables
cb_size = 50;
acc_hist = zeros(cb_size,3);
gyro_hist = zeros(cb_size,3);
counter_hist = zeros(1,cb_size);
ori_hist = zeros(cb_size,3);        % Pitch,roll,heading


% Open COM port
serial_IMU9_obj = serial(IMU9_COM,'BAUD',38400,'Timeout',0.25,'InputBufferSize',512);
fopen(serial_IMU9_obj);
disp('Initializing IMU...')

% Wait for 2 second to let the reading to settle down
pause(3)

% Clear received message in buffer
while(serial_IMU9_obj.BytesAvailable > 0)
    fscanf(serial_IMU9_obj);
end

pause(0.12)
[acc, gyro, mag, counter, IMU_available, IMU_error] = IMU9_Read(serial_IMU9_obj, acc_cali, gyro_cali, acc, gyro, mag, counter);
pause(0.12)
[acc, gyro, mag, counter, IMU_available, IMU_error] = IMU9_Read(serial_IMU9_obj, acc_cali, gyro_cali, acc, gyro, mag, counter);

% Calibration, make sure the IMU is flat
disp('Calibrating IMU...')
for i = 0:(cb_size - 1)
    pause(0.12)
    [acc, gyro, mag, counter, IMU_available, IMU_error] = IMU9_Read(serial_IMU9_obj, acc_cali, gyro_cali, acc, gyro, mag, counter);
    acc_hist(cb_size - i,:) = acc;
    gyro_hist(cb_size - i,:) = gyro;
    ori_hist(cb_size - i,3) = atan2(mag(2), mag(1));
    counter_hist(cb_size - i) = counter;
end

acc_cali = mean(acc_hist) - [0,0,9.81];
gyro_cali = mean(gyro_hist);

for i = 1:cb_size
    acc_hist(i,:) = acc_hist(i,:) - acc_cali;
    gyro_hist(i,:) = gyro_hist(i,:) - gyro_cali;
end
disp('IMU initialized and calibrated')