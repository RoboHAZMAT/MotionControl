%% ======================Kalman Filter Prediction==========================
% ME 5524: Bayesian Robotics
% Gerardo Bledt & James Burton
% Spring 2014 (4/27/2014)
%
% Prediction function for the Kalman filter generalized for any data
% 
% - Returns the predicted state at time k given k-1 (xkk0) and the 
%   covariance matrix at time k given k-1 (Sxkk0).

%% ==============================Function==================================
function [xkk0, Sxkk0] = KFPrediction(St, u)
% Will predict the x vector and S covariance matrix at time k given k-1
% Prediction algorithm for the Kalman Filter as given in class
% State vector at time k given k0 = k-1 
xkk0 = St.A*St.xk0k0 + St.B*u;
% Covariance matrix at time k given k0 = k-1
Sxkk0 = St.A*St.Sxk0k0*St.A' + St.Swk0;
