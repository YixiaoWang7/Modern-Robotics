function [T]=ForKinB(J0,thetalist,M)
% Forward kinematics using body frame, obtaining the transform matrix
% Expression:
%   [T]=ForKinB(J0,thetalist,M)
% Input:
%   J0: jacobian matrix expressed in body frame at zero configuration
%   thetalist: angle list of joints
%   M: body frame {b} expressed in the base frame {s} at zero configuration
% Output:
%   T: transform matrix of {b} in {s} when thetalist applied

T=M;
for i=1:length(thetalist)
    T=T*SE3(J0(:,i),thetalist(i));
end