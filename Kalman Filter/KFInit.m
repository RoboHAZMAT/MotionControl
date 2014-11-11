%% ===============Initializes the Kalman Filter for IMU====================
% ME 5524: Bayesian Robotics
% Gerardo Bledt & James Burton
% Spring 2014 (4/26/2014)
%
% Initialization of states and covariance for vatious parameters of IMU 
% data to be used in the Kalman Filter

%% ==========================Initialization================================
ori_KF_hist = ori_hist;
IMU_dt = 0.12;

%% Initialize Kalman Filter for pitch
pitch_St.xk0k0 = 0;                     % Initial pitch
pitch_St.Sxk0k0 = (5*pi/180)^2;         % Initial conv of pitch
pitch_St.xkk0 = pitch_St.xk0k0;       
pitch_St.Sxkk0 = pitch_St.Sxk0k0;
pitch_St.A = 1;  pitch_St.B = 1;  pitch_St.C = 1;    % KF parameters
pitch_St.Swk0 = (10*IMU_dt*pi/180)^2;  % Prediction (gyro) conv (deg/s * s)
pitch_St.Svk = NaN;                    % Correction (acc) conv (deg)

%% Initialize Kalman Filter for roll
roll_St.xk0k0 = 0;                     % Initial roll
roll_St.Sxk0k0 = (5*pi/180)^2;         % Initial conv of roll
roll_St.xkk0 = roll_St.xk0k0;         
roll_St.Sxkk0 = roll_St.Sxk0k0;
roll_St.A = 1;  roll_St.B = 1;  roll_St.C = 1;  % KF parameters
roll_St.Swk0 = (10*IMU_dt*pi/180)^2;   % Prediction (gyro) conv (deg/s * s)
roll_St.Svk = NaN;                     % Correction (acc) conv (deg)

%% Initialize Kalman Filter for heading corrected by compass
heading_St.xk0k0 = 0;                   % Initial heading
heading_St.Sxk0k0 = 100*pi/180^2;       % Initial conv of heading
heading_St.xkk0 = heading_St.xk0k0;   
heading_St.Sxkk0 = heading_St.Sxk0k0;
heading_St.A = 1; heading_St.B = 1; heading_St.C = 1;    % KF parameters
heading_St.Swk0 = (10*IMU_dt*pi/180)^2;  % Prediction (gyro) conv (deg/s*s)
heading_St.Svk = (1*pi/180)^2;           % Correction (compass) conv (deg)
heading_St.xkk = heading_St.xk0k0;

%% Initialize Kalman Filter for heading corrected by compass
SLAM_heading_St.xk0k0 = 0;                  % Initial heading
SLAM_heading_St.Sxk0k0 = 1*pi/180^2;        % Initial conv of heading
SLAM_heading_St.xkk0 = SLAM_heading_St.xk0k0;   
SLAM_heading_St.Sxkk0 = SLAM_heading_St.Sxk0k0;
SLAM_heading_St.A = 1; SLAM_heading_St.B = 1; SLAM_heading_St.C = 1; % KF 
SLAM_heading_St.Swk0 = (10*IMU_dt*pi/180)^2; % Prediction (gyro)
SLAM_heading_St.Svk = (0.5*pi/180)^2;        % Correction (SLAM) conv (deg)
SLAM_heading_St.xkk = SLAM_heading_St.xk0k0;

%% Initialize Kalman Filter for vehicle position
x_St.xk0k0 = [0; 0];                  % Initial position in m
x_St.Sxk0k0 = [100^2,0;0,100^2];      % Initial conv of heading
x_St.A = [1,0;0,1]; x_St.B = [1,0;0,1]; x_St.C = [1,0;0,1];
x_St.Swk0 = [1,0;0,1];
x_St.Swk0 = (10*IMU_dt*pi/180)^2;     % Prediction (gyro) conv (deg/s * s)
x_St.Svk = (1.5*pi/180)^2;            % Correction (compass) conv (deg)
% Declare Prediction conv just for memory allocation. 
% It will be changed in main loop
x_St.Swk0 = [1,0; 0,1];               
% Declare Correction conv just for memory allocation. 
% It will be changed in main loop.
x_St.Svk = [(50^2)/2,0; 0,(50^2)/2];   
x_St.zk = [0,0];                       % Position in m observed by GPS
x_St.u = [0;0];                        % Ctrl action in m/s
% Corrected heading in radian in cartesian coordinate
th = 0;              
x_St.xkk = x_St.xk0k0;