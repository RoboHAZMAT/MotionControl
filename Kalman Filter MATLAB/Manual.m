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
clear all
close all

%% Initialize
COM_Init;
IMU9_Init;               % Initializing 2D compass
KF_Init;
%Ctrl_Init;              % Initializing Motor Controller and Joystick
%                          uncomment lines: 61, 62, 110-134
%GPS_Init;               % Initializing GPS
%UDP_Init;
% LiDAR_Init;
% Cam_Init;
no_plot_LiDAR = 0;

% %Plot LiDAR
% if no_plot_LiDAR == 0
%     fig_LiDAR = figure('Name','LiDAR');
%     plot_LiDAR1 = plot(LiDAR_xy(:,1),LiDAR_xy(:,2),'LineWidth',3);
%     axis equal
%     %axis([-10 10 0 20])
%     xlabel('x (m)')
%     ylabel('y (m)')
% end

%%
acc = zeros(1,3);
gyro = zeros(1,3);
mag = zeros(1,3);

acc_temp = zeros(1,3);
gyro_temp = zeros(1,3);
mag_temp = zeros(1,3);
counter_previous = inf;

ctrl_error = 0;         % 0 if Controller on the robot working perfectly
mag_error = 0;          % 0 if 2D Compass working perfectly
GPS_error = 0;          % 0 if GPS is working perfectly and able to locate itself in 2D/3D
GPS_available = 0;      % 1 if new GPS reading is available
manual = 1;             % Manual/auto mode: 1 = manaul, 0 = auto
heading_desired = 0;    % Desired Heading in degree: 0 = north, 90 = east, -90 = west
heading = 0;            % Current Heading
speed_desired = 0;      % Desired Speed (m/s)
% P_steer = 2;
P_steer = 10;
%P_steer = 4;            % Gain for P controller for steering
Bearing = NaN;          % GPS Bearing
Speed = 0;              % GPS Speed

next_i_move = 0;

Lat = NaN;
Lon = NaN;
Sat_num = 0;
HDOP = NaN;
Alt = NaN;

%auto_ctrl = ctrl;
%auto_ctrl.manual = 0;
speed_error_I = 0;
origin = [-79.35529,36.57843];
GPS_pos_desired = [-79.35529,36.57843]; % Desired GPS position in degree
%GPS_waypts = [-79.35529, 36.57843; -79.35543, 36.57794; -79.35576, 36.57780; -79.35598, 36.5783; -79.35529, 36.57843];
GPS_waypts = [-79.35529, 36.57843; -79.35519, 36.57803;]; % -79.35576, 36.57786; -79.355946, 36.57814; -79.35529, 36.57843];

% Variables for collision warning and E-stop
collision_E_stop = 0;
collision_warning = 0;


% Flags
comm_error = 0;
new_reading = 0;
new_IMU9_reading = 0;
new_LiDAR_reading = 0;
new_GPS_reading = 0;
new_Bearing_reading = 0;
new_Speed_reading = 0;

% Time-related Variables
t_last_ctrl_send = 0;   % Time of sending last GPS message
t_last_GPS_Rx = 0;      % Time of receiving last GPS message
t_last_Mag_Rx = 0;      % Time of receiving last compass message
t_rx_last = 0;          % Time of receiving last UDP message
t_main_last = 0;        % Time of last main loop running
t_LiDAR_last = 0;       % Time of receiving last LiDAR message
t = 0;
i = 0;
tic;

% timer1 = timer('TimerFcn','Timerfnc1_09272010', 'Period', 0.24, 'BusyMode','queue','ErrorFcn','Errorfnc1','ExecutionMode','fixedRate','Name','Timer1');

%pause(0.2)
%start(timer1)
%pause(0.2)
%count = 1;
%% Main While Loop
while (1)
    t = toc;
    dt_main = t - t_main_last;
    t_main_last = t;
    
    i = i + 1;

%     %% Read Joy
%     [ctrl.ctrl_left, ctrl.ctrl_right, ctrl.manual] = ...
%         Joy_Read(joy, ctrl.manual, ctrl.ctrl_left, ctrl.ctrl_right);
%     % Set saturations/limit
%     ctrl.ctrl_left = min(max(ctrl.ctrl_left,50),130);
%     ctrl.ctrl_right = min(max(ctrl.ctrl_right,50),130);      
%     %         %% Read Encoder
%     %         if serial_Ctrl_obj.BytesAvailable > 2
%     %             [left_encoder, right_encoder, encoder_error] = Encoder_Read(serial_Ctrl_obj);
%     %             disp(['Left Encoder: ',num2str(left_encoder,'%5.5u'),' ,',' Right Encoder: ',num2str(right_encoder,'%5.5u')])
%     %         end
 
    %% IMU Data
    % Reads the IMU data
    [acc, gyro, mag, counter, IMU_available, IMU_error] =...
        IMU9_Read(serial_IMU9_obj, acc_cali, gyro_cali, acc, ...
        gyro, mag, counter);
    
    % Prints raw IMU data to the Command Window
    fprintf(' Acc: %6.2f %6.2f %6.2f | Gyro: %7.2f %7.2f %7.2f | Mag: %7.2f %7.2f %7.2f\n', ...
        acc(1),acc(2),acc(3),gyro(1),gyro(2),gyro(3),mag(1),mag(2),mag(3))
    
    %% LiDAR Data
%     read_LiDAR;
%     x = LiDAR_xy(:,1);
%     y = LiDAR_xy(:,2);
%     x = x/100;
%     y = y/100;
%     if no_plot_LiDAR == 0
%        set(plot_LiDAR1, 'XData', x, 'YData', y);
%     end
%     
%     %% Manual Control
% %     ctrl_error = Ctrl_Send(ctrl.ctrl_left, ctrl.ctrl_right, serial_Ctrl_obj);
%     disp(['Manual Control = [',num2str(ctrl.ctrl_left),',',num2str(ctrl.ctrl_right),']']);
%     heading_desired = heading;
%     %t_last_ctrl_send = t;
%     %ctrl.ctrl_left = 180;
%     %auto_ctrl = ctrl;
%     %auto_ctrl.manual = 0;
%     speed_error_I = 0;
% 
%     % Send control actions
%     %ctrl = auto_ctrl;
%     ctrl_error = Ctrl_Send(ctrl.ctrl_left, ctrl.ctrl_right, serial_Ctrl_obj);
%     t_last_ctrl_send = t;
%       
    pause(0.05)

end