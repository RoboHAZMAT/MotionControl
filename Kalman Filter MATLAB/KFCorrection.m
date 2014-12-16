%% ======================Kalman Filter Correction==========================
% ME 5524: Bayesian Robotics
% Gerardo Bledt & James Burton
% Spring 2014 (4/27/2014)
%
% Correction function for the Kalman filter generalized for any data
%
% - Returns the corrected state at time k given k (xkk) and the covariance
%   matrix at time k given k (Sxkk)

%% ==============================Function==================================
function [xkk, Sxkk] = KFCorrection(St, zk)

% Creates the Identity matrix 
I = diag(ones(1,length(St.xkk0)));
% Computation of the Kalman Gain
K = St.Sxkk0*St.C'*(St.C*St.Sxkk0*St.C'+St.Svk)^-1;

% Correction algorithm for the Kalman Filter as given in class
% state vector at time k given k
xkk = St.xkk0 + K*(zk - St.C*St.xkk0);
% Covariance matrix at time k given k
Sxkk = (I - K*St.C)*St.Sxkk0;

