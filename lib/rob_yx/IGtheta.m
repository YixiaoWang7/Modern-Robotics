function [G]=IGtheta(w,theta)
% Generate the inverse of G matrix
% Expression:
%   [G]=IGtheta(w,theta)
% Input:
%   w: angular axis
%   theta: angle scale
% Output:
%   IG: inverse of G which is a part of transform matrix from se3 to SE3

G=1/theta*eye(3)-1/2*CrossM(w)+(1/theta-1/2*cot(theta/2))*CrossM(w)^2;