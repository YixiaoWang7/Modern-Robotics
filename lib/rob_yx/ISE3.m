function [V]=ISE3(T)
% Transform from SE3 to se3 and then to twist
% Expression:
%   [V]=ISE3(T)
% Input:
%   T: transform matrix
% Output:
%   V: twist

R=T(1:3,1:3);
[w,theta]=IRotM(R);
p=T(1:3,4);
if theta~=0
    v=IGtheta(w,theta)*p;
    V=[w;v]*theta;
else
    w=[0;0;0];
    v=p;
    V=[w;v];
end
