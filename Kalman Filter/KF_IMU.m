%% ======================Kalman Filter Calculation=========================
% ME 5524: Bayesian Robotics
% Gerardo Bledt & James Burton
% Spring 2014 (5/1/2014)
%
% Calculates the Kalman Filter based on the IMU measurements
% Gyroscope is used for Prediction
% Magnetometer is used for the Correction

%% ==============================Function==================================
function [pitch, roll, heading, pitch_St, roll_St, heading_St] = ...
    KF_IMU(pitch_St, roll_St, heading_St, dt, acc, gyro, mag)
%% ======================Calculate Pitch and Roll==========================
total_acc = sqrt(sum(acc.^2));
pitch = -asin(acc(1)/total_acc);
roll = -asin(acc(2)/total_acc);
if total_acc > 10.2
    pitch_St.Svk = (2*pi/180)^2 + max(acos(9.81/total_acc))^2;
elseif total_acc > 9
    pitch_St.Svk = (2*pi/180)^2;
else
    pitch_St.Svk = (2*pi/180)^2 + max(acos(total_acc/9.81))^2;
end
roll_St.Svk = pitch_St.Svk;

%% =======================Kalman filter for pitch==========================
[pitch_St.xkk, pitch_St.Sxkk] = KFCorrection(pitch_St, pitch);
pitch_St.xkk = max(min(pitch_St.xkk,pi/2),-pi/2); % Saturation
pitch_St.xk0k0 = pitch_St.xkk; pitch_St.Sxk0k0 = pitch_St.Sxkk;
[pitch_St.xkk0, pitch_St.Sxkk0] = ...
    KFPrediction(pitch_St, gyro(1)*pi/180*dt);

%% ========================Kalman filter for roll==========================
[roll_St.xkk, roll_St.Sxkk] = KFCorrection(roll_St, roll);
roll_St.xkk = max(min(roll_St.xkk,pi/2),-pi/2);   % Saturation
roll_St.xk0k0 = roll_St.xkk; roll_St.Sxk0k0 = roll_St.Sxkk;
[roll_St.xkk0, roll_St.Sxkk0] = KFPrediction(roll_St, gyro(2)*pi/180*dt);

%% =========================Magnetometer Data==============================
% Pan/Tilt compensated compass
% Tilt compensated Magnetic filed X:
MAG_X = mag(1) * cos(pitch_St.xkk) + mag(3) * sin(pitch_St.xkk);
% Tilt compensated Magnetic filed Y:
MAG_Y = mag(2) * cos(roll_St.xkk) - mag(1)*sin(pitch_St.xkk)*...
    sin(roll_St.xkk) + mag(3)*cos(pitch_St.xkk)*sin(roll_St.xkk);

% Magnetic Heading
heading = atan2(MAG_Y, MAG_X);
heading = heading - 8*pi/180;
if heading > pi
    heading = heading - 2*pi;
elseif heading < -pi
    heading = heading + 2*pi;
end

%% =====================Kalman Filter for Heading==========================
if (heading_St.xkk0 - heading) > pi
    heading_St.xkk0 = heading_St.xkk0 - 2*pi;
elseif (heading_St.xkk0 - heading) < -pi
    heading_St.xkk0 = heading_St.xkk0 + 2*pi;
end
[heading_St.xkk, heading_St.Sxkk] = KFCorrection(heading_St, heading);
% Create a looping algorithm to bound the heading within -pi to pi
if heading_St.xkk > pi
    heading_St.xkk = heading_St.xkk - 2*pi;
elseif heading_St.xkk < -pi
    heading_St.xkk = heading_St.xkk + 2*pi;
end
heading_St.xk0k0 = heading_St.xkk; heading_St.Sxk0k0 = heading_St.Sxkk;
[heading_St.xkk0, heading_St.Sxkk0] = ...
    KFPrediction(heading_St, -gyro(3)*pi/180*dt);