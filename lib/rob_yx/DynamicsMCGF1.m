function [M,C,G,F]=DynamicsMCGF1(thetalist,dthetalist,g,Ftip,Mlist,Glist,Slist)
% Caculate mass matrix M(theta), H(theta,dtheta), F, which is F=J'*Ftip
% Directly, in the rebuilt matrix form
% Expression:
%   [M,C,G,F]=DynamicsMCGF1(thetalist,dthetalist,g,Ftip,Mlist,Glist,Slist)
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




%compute Amatrix, Gmatrix, advamatrix, Wmatrix, Lmatrix
Amatrix=zeros(6*n,n);
Gmatrix=zeros(6*n,6*n);
advamatrix=zeros(6*n,6*n);
Wmatrix=zeros(6*n,6*n);
Lmatrix=eye(6*n);
for i=1:n
    Amatrix(6*i-5:6*i,i)=Alist(:,i);
    Gmatrix(6*i-5:6*i,6*i-5:6*i)=Glist(:,:,i);
    advamatrix(6*i-5:6*i,6*i-5:6*i)=adv(Alist(:,i)*dthetalist(i));
    if i<n
        Wmatrix(6*i+1:6*i+6,6*i-5:6*i)=AdT(Tlist(:,:,i+1));
    end
end

for i=1:n-1
    tempT=eye(4);
    for j=i+1:n
        tempT=Tlist(:,:,j)*tempT;
        Lmatrix(6*j-5:6*j,6*i-5:6*i)=AdT(tempT);
    end
end

%compute twistbase, dtwistbase, Ftipmatrix;
twistbase=zeros(6*n,1);
dtwistbase=zeros(6*n,1);
Ftipmatrix=zeros(6*n,1);
twistbase(1:6,1)=AdT(Tlist(:,:,1))*[0 0 0 0 0 0]';
dtwistbase(1:6,1)=AdT(Tlist(:,:,1))*[0;0;0;-g];
Ftipmatrix(end-5:end,1)=(AdT(Tlist(:,:,end)))'*Ftip;

twistmatrix=Lmatrix*(Amatrix*thetalist+twistbase);

%compute advvmatrix
advvmatrix=zeros(6,6);
for i=1:n
    advvmatrix(6*i-5:6*i,6*i-5:6*i)=adv(twistmatrix(6*i-5:6*i,1));
end

M=Amatrix'*Lmatrix'*Gmatrix*Lmatrix*Amatrix;
C=-Amatrix'*Lmatrix'*(Gmatrix*Lmatrix*advamatrix*Wmatrix+advvmatrix'*Gmatrix)*Lmatrix*Amatrix*dthetalist;
G=Amatrix'*Lmatrix'*Gmatrix*Lmatrix*dtwistbase;

Tend0=eye(4,4);%Tn+1,0
for i=1:n+1
    Tend0=Tlist(:,:,i)*Tend0;
end
F=(AdT(Tend0)*jacoRobS(Slist,thetalist))'*Ftip;







