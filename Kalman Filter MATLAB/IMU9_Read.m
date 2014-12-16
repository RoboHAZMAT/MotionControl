function [acc, gyro, mag, counter, IMU_available, IMU_error] = IMU9_Read(serial_IMU9_obj, acc_cali, gyro_cali, acc, gyro, mag, counter)

try
    % Read serial message
    while(serial_IMU9_obj.BytesAvailable > 0)
        str_IMU9 = fscanf(serial_IMU9_obj);
    end
    
    % Find the positions of all '$' and '#'
    c1_pos = strfind(str_IMU9, '$');
    c2_pos = strfind(str_IMU9, '#');
    
    % Get the positions of the first '$' and '#'
    if (length(c1_pos) > 1)
        c1_pos = c1_pos(1)
    end
    if (length(c2_pos) > 1)
        c2_pos = c2_pos(1)
    end
    
    % Check if the message has any error
    if (isempty(c1_pos) | isempty(c2_pos) | c2_pos < c1_pos)
        IMU_available = 0;
        IMU_error = 0;
    else
        % Read the message and convert it to numbers
        str_IMU9 = str_IMU9(c1_pos + 1 : c2_pos - 1);
        c3_pos = strfind(str_IMU9, ',');
        
        acc(1) = str2num(str_IMU9(strfind(str_IMU9, 'ax') + 2: c3_pos(1) - 1));
        acc(2) = str2num(str_IMU9(strfind(str_IMU9, 'ay') + 2: c3_pos(2) - 1));
        acc(3) = str2num(str_IMU9(strfind(str_IMU9, 'az') + 2: c3_pos(3) - 1));
        gyro(3) = str2num(str_IMU9(strfind(str_IMU9, 'gx') + 2: c3_pos(4) - 1));
        gyro(1) = str2num(str_IMU9(strfind(str_IMU9, 'gy') + 2: c3_pos(5) - 1));
        gyro(2) = str2num(str_IMU9(strfind(str_IMU9, 'gz') + 2: c3_pos(6) - 1));
        mag(1) = str2num(str_IMU9(strfind(str_IMU9, 'mx') + 2: c3_pos(7) - 1));
        mag(2) = str2num(str_IMU9(strfind(str_IMU9, 'my') + 2: c3_pos(8) - 1));
        mag(3) = str2num(str_IMU9(strfind(str_IMU9, 'mz') + 2: c3_pos(9) - 1));
        counter = str2num(str_IMU9(strfind(str_IMU9, 'c') + 1: c3_pos(10) - 1));
        
        % Convert the numbers to SI units
        acc = (acc.*(9.81*16/32768))/0.8715 - acc_cali;         %m/s^2
        gyro = (gyro.*0.93/8) - gyro_cali;                      %deg/s
        %gyro = (gyro.*0.93/8) - gyro_cali;                      %deg/s
        
        % Eliminate any too large readings
        for k = 1:3
            if ~isnan(gyro(k))
                if gyro(k) > 1200 || gyro(k) < -1200
                    gyro(k) = NaN;
                end
            end
            if ~isnan(acc(k))
                if acc(k) > 156.96 || acc(k) < -156.96
                    acc(k) = NaN;
                end
            end
        end
        
        % Set IMU status
        IMU_available = 1;
        IMU_error = 0;
        
    end
    
catch
    IMU_available = 0;
    IMU_error = 1;
end
        