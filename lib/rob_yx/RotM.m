function [R]=RotM(w,theta)
% Generate rotation matrix through angular axis and angle theta
% Expression:
%   [R]=RotM(w,theta)
% Input:
%   w: 3x1 normalized angular vector
%   theta: angle scale
% Output:
%   R: 3x3 rotation matrix

R=eye(3)+sin(theta)*CrossM(w)+(1-cos(theta))*CrossM(w)^2;
     