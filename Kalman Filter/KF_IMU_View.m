%% ==================Kalman Filter: IMU Visualization======================
% ME 5524: Bayesian Robotics
% Gerardo Bledt & James Burton
% Spring 2014 (4/28/2014)
%
% 1. Initializes the Com ports, IMU, and Kalman Filter. 
%
% 2. Sets up the plots for the acceletometer, gyroscope, orientation, and 
%    KF orientation.
%
% 3. Includes a visualization comparing IMU estimated heading from the 
%    Kalman Filter and a direct integration of rotational velocity. Using
%    both a spherical object and a RobotBody object.
%
% 4. Reads IMU data and calculates the Kalman Filter for the IMU data with 
%    the KF_IMU.m function if it is available and with no error. Stores the
%    data in history buffers where entry 1 is the most recent reading. 
%
% 5. Performs a Direct Integration (DI) the raw gyroscope data for an 
%    estimate of the angle and then finds the KF estimate for the heading.
%
% 6. Rotates the sphere visualization of both the DI and KF estimates.
%    Finds the rotation matricies that are associated with each estimate
%    and uses that to calculate new vertex locations for the RobotBody.
%
%
%  Created MATLAB Functions Used:
%
%   Initialization
%   - Com_Init.m
%   - IMU9_Init
%   - KFInit.m
%  
%   Calculation of Kalman Filter
%   - KF_IMU.m
%     - KFPrediction.m
%     - KFCorrection.m
%  
%   Visualization of Robot
%   - RobotBody.m
%   - RobotBodyVisual.m

%% ===========================IMU & KF Setup===============================
Close_COM; close all; clear all; clc;

% Initialize IMU and do calibration, make sure IMU is flat
delete(instrfindall)
%COM_Init;      % COM port initialization
%IMU9_Init;     % IMU initialization
%KFInit;        % KF initialization
IMU_dt = 0.12; % change in KF_Init
FontSize = 14;
LineWidth = 2;

%% ==========================Setup Data Plots==============================
% Accelerometer Plot Setup
% figure_acc = figure('Name','Accelerometer'); hold on;
% plot_acc(1) = plot(counter_hist.*0.120, acc_hist(:,1)','-b');
% plot_acc(2) = plot(counter_hist.*0.120, acc_hist(:,2)','-g');
% plot_acc(3) = plot(counter_hist.*0.120, acc_hist(:,3)','-r');
% xlabel('Time (s)'); ylabel('Acceleration (ms^-2)')
% ylim([-20 20]); legend('x','y','z');

% Gyroscope Plot Setup
figure_gyro = figure('Name','Gyro'); hold on;
title('Raw Gyroscope Data', 'FontSize', FontSize)
plot_gyro(1) = plot(counter_hist.*0.120, gyro_hist(:,1)','-b',...
    'LineWidth', LineWidth);
plot_gyro(2) = plot(counter_hist.*0.120, gyro_hist(:,2)','-g',...
    'LineWidth', LineWidth);
plot_gyro(3) = plot(counter_hist.*0.120, gyro_hist(:,3)','-r',...
    'LineWidth', LineWidth);
xlabel('Time (s)', 'FontSize', FontSize); 
ylabel('Angular Velocity (deg/s)', 'FontSize', FontSize);
ylim([-180 180]); legend('x', 'y', 'z'); grid;

% Orientation Plot Setup
figure_ori = figure('Name','Orientation'); hold on;
title('Orientation Estimate', 'FontSize', FontSize)
% Orientation Plot
plot_ori(1) = plot(counter_hist.*0.120, 180/pi*ori_hist(:,1)',':b',...
    'LineWidth', LineWidth);
plot_ori(2) = plot(counter_hist.*0.120, 180/pi*ori_hist(:,2)',':g',...
    'LineWidth', LineWidth);
plot_ori(3) = plot(counter_hist.*0.120, 180/pi*ori_hist(:,3)',':r',...
    'LineWidth', LineWidth);
% Kalman Filter Orientation Plot
plot_KF_ori(1) = plot(counter_hist.*0.120, 180/pi*ori_KF_hist(:,1)',...
    '-b', 'LineWidth', LineWidth);
plot_KF_ori(2) = plot(counter_hist.*0.120, 180/pi*ori_KF_hist(:,2)',...
    '-g', 'LineWidth', LineWidth);
plot_KF_ori(3) = plot(counter_hist.*0.120, 180/pi*ori_KF_hist(:,3)',...
    '-r', 'LineWidth', LineWidth);
xlabel('Time (s)', 'FontSize', FontSize); 
ylabel('Angle (deg)', 'FontSize', FontSize); ylim([-180 180]);
legend('Pitch','Roll','Heading','KF Pitch','KF Roll','KF Heading'); grid;

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
% Visualization of Direct Integration Heading
figure_sphere = figure('Name','Sphere'); hold on;
title('Direct Integration Orientation Estimate', 'FontSize', FontSize)
[x,y,z] = sphere(20); h = surf(x,y,z);
gyroAngle = [0,0,0]; gyroAngleLast = [0,0,0]; gyroAngleDiff = [0,0,0];

% Visualization of Kalman Filtered Heading
figure_sphereKF = figure('Name','KF Sphere'); hold on;
title('Kalman Filter Orientation Estimate', 'FontSize', FontSize)
[xKF,ykf,zKF] = sphere(20); hKF = surf(xKF,ykf,zKF);
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

%% =======================Main Running While Loop==========================
fprintf('IMU Running...');
while(1)
    % Read IMU data
    [acc, gyro, mag, counter, IMU_available, IMU_error] = ...
        IMU9_Read(serial_IMU9_obj, acc_cali, gyro_cali,...
        acc, gyro, mag, counter);
    
    % If the IMU is available and has no error, calculate the KF and plot
    if (IMU_available == 1 && IMU_error == 0)
        [pitch, roll, heading, pitch_St, roll_St, heading_St] = ...
            KF_IMU(pitch_St, roll_St, heading_St, IMU_dt*(counter ...
            - counter_hist(1)), acc, (gyro-gyro_cali), mag);
        
        % Store the readings in circular buffer where the first entry is 
        % the latest value read from the IMU
        acc_hist(2:(cb_size),:) = acc_hist(1:(cb_size-1),:);
        acc_hist(1,:) = acc;
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
%             set(plot_acc(i), 'XData', counter_hist.*0.120, 'YData',...
%                 acc_hist(:,i)');
            set(plot_gyro(i), 'XData', counter_hist.*0.120, 'YData',...
                gyro_hist(:,i)');
            set(plot_ori(i), 'XData', counter_hist.*0.120, 'YData',...
                180/pi*ori_hist(:,i)');
            set(plot_KF_ori(i), 'XData', counter_hist.*0.120, 'YData',...
                180/pi*ori_KF_hist(:,i)');
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
        set(gcf,'Color', [1.0, 1.0, 1.0]);
        
        % Rotate the sphere visualizations
        rotate(h,[0,1,0],gyroAngleDiff(1,3))
        rotate(hKF,[0,-1,0],180/pi*kfDiff(1,3))
        
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
    end
end
