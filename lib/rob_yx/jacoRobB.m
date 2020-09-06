function [J]=jacoRobB(J0,thetalist)
% Generate space jacobian from thetalist
% Expression:
%   [J]=jacoRobB(J0,thetalist)
% Input:
%   J0: jacobian matrix at zero position expressed in the base frame, or
%   end-effector frame {b}. Each column is the screw vector.
%   thetalist: angle list for all the joints
% Output:
%   J: jacobian matrix at thetalist position

n=size(J0,2);
J=zeros(6,n);
J(:,end)=J0(:,end);
for i=1:n-1
    T=eye(4);
    for j=i+1:n
        T=SE3(-J0(:,j),thetalist(j))*T;
    end
    J(:,i)=AdT(T)*J0(:,i);
    end
end


