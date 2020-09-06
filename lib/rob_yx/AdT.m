function [A]=AdT(T)
% Generate AdT
% Expression:
%   [A]=AdT(T)
% Input:
%   T: transform matrix
% Output:
%   A: AdT matrix

R=T(1:3,1:3);
p=(T(1:3,4));
A=[R zeros(3,3);CrossM(p)*R R];