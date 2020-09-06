function [T]=SE3(S,theta)
% Transform from se3 to SE3
% Expression:
%   [T]=SE3(S,theta)
% Input:
%   S: 6x1 screw vector
%   theta: angle scale
% Output:
%   T: Transform matrix

w=S(1:3);
v=S(4:6);
R=RotM(w,theta);
G=Gtheta(w,theta);
T=zeros(4,4);
T(4,4)=1;
T(1:3,1:3)=R;
T(1:3,4)=G*[v(1);v(2);v(3)];


