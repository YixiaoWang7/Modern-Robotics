function [w,theta]=IRotM(R)
% Generate angual axis and angle scale from rotation matrix
% Expression:
%   [w,theta]=IRotM(R)
% Input:
%   R: rotation matrix
% Output:
%   w: angular axis
%   theta: angle scale
% Special case:
%   if R=I, w=[0 0 1]', theta=0

theta=acos((R(1,1)+R(2,2)+R(3,3)-1)/2);
if abs(sin(theta))>10^-10
    W=1/2/sin(theta)*(R-R');
    w=[-W(2,3) W(1,3) -W(1,2)]';
elseif R(1,1)==1&R(2,2)==1&R(3,3)==1
    theta=0;
    w=[0;0;1];
else
    theta=pi;
    for i=1:3
        if R(i,i)~=-1
            w=[R(1,i);R(2,i);R(3,i)];
            w(i)=w(i)+1;
            w=w/sqrt(2+2*R(i,i));
            break
        end
    end
end
     