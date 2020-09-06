function [wrong]=testjointlimits(con)
% jointlimit=[-2.932 +2.932;-1.117 +1.553;-2.62 2.53;-1.78 1.78;-2.89 2.89];
jointlimit=[-2.932 +2.932;-1.117 0;-2.62 2.53;-1.78 1.78;-2.89 2.89];
joint=con(4:8);%column vector
mismatch=(joint<jointlimit(:,1))+(joint>jointlimit(:,2));
if 155*cos(joint(3))+217.6*cos(joint(4))+135<0
    mismatch=mismatch+[0 0 1 1 0]';
end
wrong=find(mismatch~=0);





