function [ddthetalist] = ForDynamics(thetalist,dthetalist,taulist,g,Ftip,Mlist,Glist,Slist)
% Forward dynamics
% Expression:
%   [ddthetalist] = ForDynamics(thetalist,dthetalist,taulist,g,Ftip,Mlist,Glist,Slist)
% Input:
%   thetalist/d~: initial thetalist/1-order derivative
%   taulist: the torque taulist
%   g: acceleration of gravity
%   Ftip: external force at the end-effector
%   Mlist: 3d matrix  Mi-1,i...
%   Glist: 3d matrix spatial inertia matrix
%   Slist: screw at zero configuration expressed in base frame {s}
% output:
%   ddthetalist: joint acceleration list

%caculate the M(theta), H(theta,dtheta), F, which is F=J'*Ftip
%using inverse dynamics
[M,H,F]=DynamicsMHF2(thetalist,dthetalist,g,Ftip,Mlist,Glist,Slist);

ddthetalist=pinv(M)*(taulist-H-F);

%the error will be accumalted and become very large when calculating the
%mass matrix and h and f directly.
% [M,C,G,F]=DynamicsMCG1(thetalist,dthetalist,g,Ftip,Mlist,Glist,Slist);
% ddthetalist=pinv(M)*(taulist-C-G-F);



