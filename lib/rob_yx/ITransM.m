function [IT]=ITransM(T)
% Inverse of transform matrix 
% Expression:
%   [IT]=ITransM(T)
% Input:
%   T: 4x4 transform matrix
% Output:
%   IT: 4x4 inverse of transform matrix

R=T(1:3,1:3);
p=T(1:3,4);
IT=[R' -R'*p;0 0 0 1];