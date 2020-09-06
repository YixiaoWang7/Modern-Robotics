function [J]=jacoRobS(J0,thetalist)
% Generate space jacobian from thetalist
% Expression:
%   [J]=jacoRobS(J0,thetalist)
% Input:
%   J0: jacobian matrix at zero position expressed in the base frame {s}
%       Each column is the screw vector.
%   thetalist: angle list for all the joints
% Output:
%   J: jacobian matrix at thetalist position

J=zeros(6,6);
J(:,1)=J0(:,1);
for i=2:6
    T=eye(4);
    for j=1:i-1
        T=T*SE3(J0(:,j),thetalist(j));
    end
    J(:,i)=AdT(T)*J0(:,i);
end


