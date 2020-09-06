function [taulist] = IDynamics(thetalist,dthetalist,ddthetalist,g,Ftip,Mlist,Glist,Slist)
% Inverse dynamics
% Expression:
%   [taulist] = IDynamics(thetalist,dthetalist,ddthetalist,g,Ftip,Mlist,Glist,Slist)
% Input:
%   thetalist/d~/dd~: initial thetalist/1-order derivative/2-order
%   derivative
%   g: acceleration of gravity
%   Ftip: external force at the end-effector
%   Mlist: 3d matrix  Mi-1,i...
%   Glist: 3d matrix spatial inertia matrix
%   Slist: screw at zero configuration expressed in base frame {s}
% output:
%   taulist: the torque taulist

n=length(thetalist);

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


%compute twistlist and dtwistlist
%V0, Vn
%dV0,dVn
twistlist=zeros(6,n+1);
dtwistlist=zeros(6,n+1);
dtwistlist(4:6,1)=-g;
for i=1:n
    twistlist(:,i+1)=Alist(:,i)*dthetalist(i)+AdT(Tlist(:,:,i))*twistlist(:,i);
    dtwistlist(:,i+1)=Alist(:,i)*ddthetalist(i)+adv(twistlist(:,i+1))*Alist(:,i)*dthetalist(i)+AdT(Tlist(:,:,i))*dtwistlist(:,i);
end

%compute Flist and taulist
%F1,Fn+1
%tau1,taun
Flist=zeros(6,n+1);
taulist=zeros(n,1);
Flist(:,end)=Ftip;
for i=n:-1:1
    Flist(:,i)=(AdT(Tlist(:,:,i+1)))'*Flist(:,i+1)+Glist(:,:,i)*dtwistlist(:,i+1)-(adv(twistlist(:,i+1)))'*(Glist(:,:,i)*twistlist(:,i+1));
    taulist(i)=Flist(:,i)'*Alist(:,i);
end
