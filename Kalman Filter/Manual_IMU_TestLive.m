%% This is the main function that does autonomous exploration following
%% waypoints that the robot generates, doing self-localization, and
%% creating an occupancy map.


%% Stop and delete all timers
Clear_Timers;

%% Close ports, if they are open
delete(instrfindall)
Close_COM;

%% Clear memory and close all figures
clc
% clear all
close all

%% Initialize
COM_Init;
Ctrl_Init;
IMU9_Init; 

 % Initializing 2D compass
% KF_Init;
KFInit;
 
%                          uncomment lines: 61, 62, 110-134
%GPS_Init;               % Initializing GPS
%UDP_Init;
% LiDAR_Init;
% Cam_Init;
% no_plot_LiDAR = 0;

%Plot LiDAR
% if no_plot_LiDAR == 0
%     fig_LiDAR = figure('Name','LiDAR');
%     plot_LiDAR1 = plot(LiDAR_xy(:,1),LiDAR_xy(:,2),'LineWidth',3);
%     axis equal
%     %axis([-10 10 0 20])
%     xlabel('x (m)')
%     ylabel('y (m)')
% end

%%
% acc = zeros(1,3);
% gyro = zeros(1,3);
% mag = zeros(1,3);
%
% acc_temp = zeros(1,3);
% gyro_temp = zeros(1,3);
% mag_temp = zeros(1,3);
% counter_previous = inf;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



FontSize = 14;
LineWidth = 2;

%% ==========================Setup Data Plots==============================



% Comparison Plot Setup
figure_comp = figure('Name','Comparison'); hold on;
title('Orientation Estimate for DI and KF', 'FontSize', FontSize)
DIgyro = zeros(50,1);
plot_DIgyro = plot(counter_hist*0.120, DIgyro(:),...
    '-g', 'LineWidth', 1.5*LineWidth);
plot_KF = plot(counter_hist.*0.120, 180/pi*ori_KF_hist(:,3)',...
    '-r', 'LineWidth', 1.5*LineWidth);
xlabel('Time (s)', 'FontSize', FontSize); 
ylabel('Angle (deg)', 'FontSize', FontSize); ylim([-180 180]);
legend('DI','KF'); grid;

%% =====================Setup Sphere Visualization=========================
gyroAngle = [0,0,0]; gyroAngleLast = [0,0,0]; gyroAngleDiff = [0,0,0];

kfLast = [0,0,0]; kfDiff = [0,0,0];

%% ======================Setup Robot Visualization=========================
% Visualization of Direct Integration Heading
figure_Robot = figure('Name','Robot'); hold on;
title('Direct Integration Orientation Estimate', 'FontSize', FontSize)
[V,lims,color] = RobotBody;

% Visualization of Kalman Filtered Heading
figure_RobotKF = figure('Name','KF Robot'); hold on;
title('Kalman Filter Orientation Estimate', 'FontSize', FontSize)
[VKF,limsKF,colorKF] = RobotBody;






%%%%%%%%%%%%%%%%%





ctrl_error = 0;         % 0 if Controller on the robot working perfectly
% mag_error = 0;          % 0 if 2D Compass working perfectly
% GPS_error = 0;          % 0 if GPS is working perfectly and able to locate itself in 2D/3D
% GPS_available = 0;      % 1 if new GPS reading is available
manual = 1;             % Manual/auto mode: 1 = manaul, 0 = auto
% heading_desired = 0;    % Desired Heading in degree: 0 = north, 90 = east, -90 = west
% heading = 0;            % Current Heading
% speed_desired = 0;      % Desired Speed (m/s)
% % P_steer = 2;
% P_steer = 10;
% %P_steer = 4;            % Gain for P controller for steering
% Bearing = NaN;          % GPS Bearing
% Speed = 0;              % GPS Speed

% next_i_move = 0;

% Lat = NaN;
% Lon = NaN;
% Sat_num = 0;
% HDOP = NaN;
% Alt = NaN;

%auto_ctrl = ctrl;
%auto_ctrl.manual = 0;
% speed_error_I = 0;
% origin = [-79.35529,36.57843];
% GPS_pos_desired = [-79.35529,36.57843]; % Desired GPS position in degree
% %GPS_waypts = [-79.35529, 36.57843; -79.35543, 36.57794; -79.35576, 36.57780; -79.35598, 36.5783; -79.35529, 36.57843];
% GPS_waypts = [-79.35529, 36.57843; -79.35519, 36.57803;]; % -79.35576, 36.57786; -79.355946, 36.57814; -79.35529, 36.57843];

% Variables for collision warning and E-stop
% collision_E_stop = 0;
% collision_warning = 0;


% Flags
% comm_error = 0;
% new_reading = 0;
% new_IMU9_reading = 0;
% new_LiDAR_reading = 0;
% new_GPS_reading = 0;
% new_Bearing_reading = 0;
% new_Speed_reading = 0;

% Time-related Variables
% t_last_ctrl_send = 0;   % Time of sending last GPS message
% t_last_GPS_Rx = 0;      % Time of receiving last GPS message
% t_last_Mag_Rx = 0;      % Time of receiving last compass message
% t_rx_last = 0;          % Time of receiving last UDP message
% t_main_last = 0;        % Time of last main loop running
% t_LiDAR_last = 0;       % Time of receiving last LiDAR message
% t = 0;
% i = 0;
tic;

% timer1 = timer('TimerFcn','Timerfnc1_09272010', 'Period', 0.24, 'BusyMode','queue','ErrorFcn','Errorfnc1','ExecutionMode','fixedRate','Name','Timer1');
% 
% pause(0.2)
% start(timer1)
% pause(1)
% count = 1;serial_Ctrl_obj,'%c')

%% IMU Data Collection Initialization
% N = 220;              % Number of data points to collect
% acc = zeros(N,3);      % Initialize data arrays
% gyro = zeros(N,3);
% mag = zeros(N,3);
% time = zeros(N,1);     % Initialize time array
% 
% acc_temp = zeros(1,3); % Initialize temporary data arrays 
% gyro_temp = zeros(1,3);
% mag_temp = zeros(1,3);
% counter_previous = inf;
% finish = 0;
% index = 1;

fprintf('Collecting data...\n');
%% Main While Loop
k=0;
%while (finish == 0)
while(1)
    t = toc;
%     dt_main = t - t_main_last;
%     t_main_last = t;
    
%     i = i + 1;
    %% IMU Data Collection
%      [acc_temp, gyro_temp, mag_temp, counter, IMU_available, IMU_error]...
%          = IMU9_Read(serial_IMU9_obj, acc_cali, gyro_cali, acc_temp,...
%          gyro_temp, mag_temp, counter);
%     if counter ~= counter_previous
%         time(counter) = toc;
%         %fprintf('Elasped Time: %3.3f\n', time(counter));
%         %acc_temp = (acc_temp.*(9.81*16/32768))/0.8715 - acc_cali;         %m/s^2
%         %gyro_temp = (gyro_temp.*0.93/8);% - gyro_cali;                    %deg/s
%         
%         acc(counter,1) = acc_temp(1);
%         acc(counter,2) = acc_temp(2);
%         acc(counter,3) = acc_temp(3);
%         gyro(counter,1) = gyro_temp(1);
%         gyro(counter,2) = gyro_temp(2);
%         gyro(counter,3) = gyro_temp(3);
%         mag(counter,1) = mag_temp(1);
%         mag(counter,2) = mag_temp(2);
%         mag(counter,3) = mag_temp(3);
%         
%         index = index + 1;
%         
%         if index >= N 
%             finish = 1;
%         end
%     end
%     counter_previous = counter;
    %% Read Joy
%     [ctrl.ctrl_left, ctrl.ctrl_right, ctrl.manual] = ...
%         Joy_Read(joy, ctrl.manual, ctrl.ctrl_left, ctrl.ctrl_right);
%     % Set saturations/limit
%     ctrl.ctrl_left = min(max(ctrl.ctrl_left,50),130);
%     ctrl.ctrl_right = min(max(ctrl.ctrl_right,50),130);
    %         %% Read Encoder
    %         if serial_Ctrl_obj.BytesAvailable > 2
    %             [left_encoder, right_encoder, encoder_error] = Encoder_Read(serial_Ctrl_obj);
    %             disp(['Left Encoder: ',num2str(left_encoder,'%5.5u'),' ,',' Right Encoder: ',num2str(right_encoder,'%5.5u')])
    %         end
    
    %     %% IMU Data
    %     % Reads the IMU data
%         [acc_temp, gyro_temp, mag_temp, counter, IMU_available, IMU_error] =...
%             IMU9_Read(serial_IMU9_obj, acc_cali, gyro_cali, acc_temp, ...
%             gyro_temp, mag_temp, counter);
    %
    %     % Adds new data to the acc, gyro, and mag vectors only if the IMU
    %     % actually took a new reading
    %     if counter ~= counter_previous
    %         acc(1,1) = acc_temp(1);
    %         acc(1,2) = acc_temp(2);
    %         acc(1,3) = acc_temp(3);
    %         gyro(1,1) = gyro_temp(1);
    %         gyro(1,2) = gyro_temp(2);
    %         gyro(1,3) = gyro_temp(3);
    %         mag(1,1) = mag_temp(1);
    %         mag(1,2) = mag_temp(2);
    %         mag(1,3) = mag_temp(3);
    %     end
    %     counter_previous = counter;
    %
    %     % Prints raw IMU data to the Command Window
    %     fprintf('IMU Data: [Acc: %3.2f %3.2f %3.2f Gyro: %3.2f %3.2f %3.2f'...
    %         + 'Mag: %3.2f %3.2f %3.2f\n', ...
    %         acc(1,1), acc(1,2), acc(1,3), ...
    %         gyro(1,1), gyro(1,2), gyro(1,3), ...
    %         mag(1,1), mag(1,2), mag(1,3))
    %
    %     %% LiDAR Data
    %     read_LiDAR;
    %     x = LiDAR_xy(:,1);
    %     y = LiDAR_xy(:,2);
    %     x = x/100;
    %     y = y/100;
    %     if no_plot_LiDAR == 0
    %        set(plot_LiDAR1, 'XData', x, 'YData', y);
    %     end
    %
    %% Manual Control
    %     ctrl_error = Ctrl_Send(ctrl.ctrl_left, ctrl.ctrl_right, serial_Ctrl_obj);
    %disp(['Manual Control = [',num2str(ctrl.ctrl_left),',',num2str(ctrl.ctrl_right),']']);
    %     heading_desired = heading;
    %t_last_ctrl_send = t;
    %ctrl.ctrl_left = 180;
    %auto_ctrl = ctrl;
    %auto_ctrl.manual = 0;
    %     speed_error_I = 0;
    % Read IMU data
    [acc, gyro, mag, counter, IMU_available, IMU_error] = ...
        IMU9_Read(serial_IMU9_obj, acc_cali, gyro_cali,...
        acc, gyro, mag, counter);
    
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    
    % If the IMU is available and has no error, calculate the KF and plot
    
        [pitch, roll, heading, pitch_St, roll_St, heading_St] = ...
            KF_IMU(pitch_St, roll_St, heading_St, IMU_dt*(counter ...
            - counter_hist(1)), acc, (gyro-gyro_cali), mag);
        
        % Store the readings in circular buffer where the first entry is 
        % the latest value read from the IMU
        gyro_hist(2:(cb_size),:) = gyro_hist(1:(cb_size-1),:);
        gyro_hist(1,:) = gyro;
        DIgyro(2:(cb_size)) = DIgyro(1:(cb_size-1));
        DIgyro(1) = -(gyroAngle(1,3)+gyro(1,3)*IMU_dt);
        if (DIgyro(1) > 180)
            DIgyro(1) = DIgyro(1) - 2*180;
        elseif (DIgyro(1) < -180)
            DIgyro(1) = DIgyro(1) + 2*180;
        end
        ori_hist(2:(cb_size),:) = ori_hist(1:(cb_size-1),:);
        ori_hist(1,:) = [pitch, roll, heading];
        ori_KF_hist(2:(cb_size),:) = ori_KF_hist(1:(cb_size-1),:);
        ori_KF_hist(1,:) = [pitch_St.xkk, roll_St.xkk, heading_St.xkk];
        counter_hist(2:cb_size) = counter_hist(1:(cb_size-1));
        counter_hist(1) = counter;
        
        % Plot IMU Readings
        for i = 1:3
            % Calculates the gyro angle by direct integration
            gyroAngle(1,i) = gyroAngle(1,i) + gyro_hist(1,i)*IMU_dt;
            gyroAngleDiff(1,i) = gyroAngle(1,i) - gyroAngleLast(1,i);
            gyroAngleLast(1,i) = gyroAngle(1,i);
            % Calculates dtheta/dt for the Kalman Filtered signal
            kfDiff(1,i) = ori_KF_hist(1,i) - kfLast(1,i);
            kfLast(1,i) = ori_KF_hist(1,i);
        end
        % Plotting the comparison data
        set(plot_DIgyro, 'XData', counter_hist.*0.120,...
            'YData', DIgyro(:)');
        set(plot_KF, 'XData', counter_hist.*0.120,...
            'YData', 180/pi*ori_KF_hist(:,3)');
        
        % Calculate the Rotation Matricies
        % Rotation angle & matrix for DI
        th = -gyroAngleDiff(1,3);
        R = [cosd(th),sind(th),0;...
            -sind(th),cosd(th),0;...
            0, 0, 1];
        % Rotation angle & matrix for KF
        thKF = 180/pi*kfDiff(1,3);
        RKF = [cosd(thKF),sind(thKF),0;...
            -sind(thKF),cosd(thKF),0;...
            0, 0, 1];
        
        % Calculate new vertex locations after rotation
        for i = 1:8
            % Rotation for DI
            V(:,i) = R*V(:,i);
        end
        for i = 1:8
            % Rotation for KF
            VKF(:,i) = RKF*VKF(:,i);
        end
        
        % Plot the visualizations of DI and KF Robot body
        M = 0.75; % Scaling factor for the fonts
        % Update the DI robot body visualization
        clf(figure_Robot);
        set(0, 'currentfigure', figure_Robot);
        RobotBodyVisual(V,lims,color);
        set(gcf,'Color', [0.9, 0.9, 0.9]);
        title('Direct Integration Orientation Estimate',...
            'FontSize', FontSize)
        xlabel('X (m)', 'FontSize', M*FontSize); 
        ylabel('Y (m)', 'FontSize', M*FontSize);
        zlabel('Z (m)', 'FontSize', M*FontSize);
        % Update the KF robot body visualization
        clf(figure_RobotKF);
        set(0, 'currentfigure', figure_RobotKF);
        RobotBodyVisual(VKF,limsKF,colorKF);
        set(gcf,'Color', [0.9, 0.9, 0.9]);
        title('Kalman Filter Orientation Estimate', 'FontSize', FontSize)
        xlabel('X (m)', 'FontSize', M*FontSize); 
        ylabel('Y (m)', 'FontSize', M*FontSize);
        zlabel('Z (m)', 'FontSize', M*FontSize);
        
        % Recalibrate the IMU
        gyro_cali = Gyro_recali(gyro_hist, ori_hist, IMU_dt);
        
        % Draw the plots and visualizations
        drawnow;
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    % Send control actions
    %ctrl = auto_ctrl;
    KFheadin = 180/pi*ori_KF_hist(1,3);
    if (KFheadin > 40)
        ctrl.ctrl_right = 105;
        ctrl.ctrl_left = 75;
    elseif (KFheadin < -40)
       ctrl.ctrl_left = 105; 
       ctrl.ctrl_right = 75;
    end
    ctrl.ctrl_left
    KFheadin
    ctrl_error = Ctrl_Send(ctrl.ctrl_left, ctrl.ctrl_right, serial_Ctrl_obj);
    %     t_last_ctrl_send = t;
    
    pause(0.05)
    if k==0
        fscanf(serial_Ctrl_obj,'%c')
        k = 2
        pause(1);
    end
end
Ctrl_Send(90, 90, serial_Ctrl_obj);
%% Write Output Files
fileout = fopen('AccOutput.txt','wt+','n','UTF-8');
for i=1:length(acc(:,1))
    fprintf(fileout,'\n%3.3f %3.3f %3.3f %3.3f\n',time(i),acc(i,1),acc(i,2),acc(i,3));
end
%fwrite(fileout,acc,'double',0,'n');
fclose(fileout);

fileout = fopen('GyroOutput.txt','wt+','n','UTF-8');
for i=1:length(acc(:,1))
    fprintf(fileout,'\n%3.3f %3.3f %3.3f %3.3f\n',time(i),gyro(i,1),gyro(i,2),gyro(i,3));
end
fclose(fileout);

fileout = fopen('MagnoOutput.txt','wt+','n','UTF-8');
for i=1:length(acc(:,1))
    fprintf(fileout,'\n%3.3f %3.3f %3.3f %3.3f\n',time(i),mag(i,1),mag(i,2),mag(i,3));
end
fclose(fileout);

delete(instrfindall);