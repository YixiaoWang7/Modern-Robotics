function [T]=TransM(R,p)
% Transform matrix using rotation matrix R and position p
% Expression:
%   [T]=TransM(R,p)
% Input: 
%   R: 3x3 rotation matrix
%   p: 3x1 position vector
% Output:
%   T: 4x4 transform matrix

T=[R p;0 0 0 1];