function [W]=CrossM(w)
% Generate skew symmetric matrix
% Expression:
%   [W]=CrossM(w)
% Input:
%   w: angular axis
% Output:
%   W: skew symmetric matrix

W=[0 -w(3) w(2);w(3) 0 -w(1);-w(2) w(1) 0];