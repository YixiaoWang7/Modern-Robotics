function [M,H,F]=DynamicsMHF2(thetalist,dthetalist,g,Ftip,Mlist,Glist,Slist)
% Caculate mass matrix M(theta), H(theta,dtheta), F, which is F=J'*Ftip
% using inverse dynamics
% Expression:
%   [M,H,F]=DynamicsMHF2(thetalist,dthetalist,g,Ftip,Mlist,Glist,Slist)
% Input:
%   thetalist/d~: initial thetalist/1-order derivative
%   g: acceleration of gravity
%   Ftip: external force at the end-effector
%   Mlist: 3d matrix  Mi-1,i...
%   Glist: 3d matrix spatial inertia matrix
%   Slist: screw at zero configuration expressed in base frame {s}
% output:
%   M: mass matrix
%   H: h(theta, dtheta)
%   F: f(Ftip,J)

n=length(thetalist);
M=zeros(6,n);
H=zeros(6,1);
F=zeros(6,1);

for i=1:n
    dd=zeros(6,1);
    dd(i)=1;
    M(:,i)=IDynamics(thetalist,zeros(6,1),dd,zeros(3,1),zeros(6,1),Mlist,Glist,Slist);
end


H=IDynamics(thetalist,dthetalist,zeros(6,1),g,zeros(6,1),Mlist,Glist,Slist);




%compute Milist: M0,Mn+1
NofMlist=size(Mlist,3);
Milist=zeros(4,4,NofMlist+1);
Milist(:,:,1)=eye(4);%M0
for i=1:NofMlist
    for j=1:i
        Milist(:,:,i+1)=Milist(:,:,i)*Mlist(:,:,i);
    end
end
%compute Alist:theta1,thetan
Alist=zeros(6,n);
for i=1:n
    Alist(:,i)=AdT(ITransM(Milist(:,:,i+1)))*Slist(:,i);
end
%compute Tlist: Ti,i-1
%T1,0 to Tn+1,n
Tlist=zeros(4,4,NofMlist);
for i=1:NofMlist-1
    %ITransM(Milist(:,:,i+1))*Milist(:,:,i) means Mi,i-1
    Tlist(:,:,i)=SE3(Alist(:,i),-thetalist(i))*ITransM(Milist(:,:,i+1))*Milist(:,:,i);
end
%the end effector frame is fixed to the nth frame
Tlist(:,:,end)=ITransM(Milist(:,:,end))*Milist(:,:,end-1);
Tend0=eye(4,4);%Tn+1,0
for i=1:n+1
    Tend0=Tlist(:,:,i)*Tend0;
end
F=(AdT(Tend0)*jacoRobS(Slist,thetalist))'*Ftip;