function [G]=Gtheta(w,theta)
% Generate G matrix
% Expression:
%   [G]=Gtheta(w,theta)
% Input:
%   w: angular axis
%   theta: angle scale
% Output:
%   IG: 3x3 matrix G which is a part of transform matrix from se3 to SE3

G=theta*eye(3)+(1-cos(theta))*CrossM(w)+(theta-sin(theta))*CrossM(w)^2;