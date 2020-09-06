function [Ncon]=TrajectoryGenerator(Tse0, Tsc0, Tsc1, Tceg, Tcest, k, method)
addpath('../lib/mr')
% method==0:screw;method==1:cartesian
% ScrewTrajectory(Xstart,Xend,Tf,N,method)
% CartesianTrajectory

dt=0.01;
T1=10;
Ts=2;
Tg=0.63;
T2=10;
if method==0
Ncon=[];
traj=ScrewTrajectory(Tse0, Tsc0*Tcest, T1, T1/dt*k+1, k);
Ncon=[Ncon; T2vec(traj,0)];

traj=ScrewTrajectory(Tsc0*Tcest, Tsc0*Tceg, Ts, Ts/dt*k+1, k);
Ncon=[Ncon; T2vec(traj,0)];

traj=ScrewTrajectory(Tsc0*Tceg, Tsc0*Tceg, Tg, Tg/dt*k+1, k);
Ncon=[Ncon; T2vec(traj,1)];

traj=ScrewTrajectory(Tsc0*Tceg, Tsc0*Tcest, Ts, Ts/dt*k+1, k);
Ncon=[Ncon; T2vec(traj,1)];

traj=ScrewTrajectory(Tsc0*Tcest, Tsc1*Tcest, T2, T2/dt*k+1, k);
Ncon=[Ncon; T2vec(traj,1)];

traj=ScrewTrajectory(Tsc1*Tcest, Tsc1*Tceg, Ts, Ts/dt*k+1, k);
Ncon=[Ncon; T2vec(traj,1)];

traj=ScrewTrajectory(Tsc1*Tceg, Tsc1*Tceg, Tg, Tg/dt*k+1, k);
Ncon=[Ncon; T2vec(traj,0)];

traj=ScrewTrajectory(Tsc1*Tceg, Tsc1*Tcest, Ts, Ts/dt*k+1, k);
Ncon=[Ncon; T2vec(traj,0)];


else
Ncon=[];
traj=CartesianTrajectory(Tse0, Tsc0*Tcest, T1, T1/dt*k+1, k);
Ncon=[Ncon; T2vec(traj,0)];

traj=CartesianTrajectory(Tsc0*Tcest, Tsc0*Tceg, Ts, Ts/dt*k+1, k);
Ncon=[Ncon; T2vec(traj,0)];

traj=CartesianTrajectory(Tsc0*Tceg, Tsc0*Tceg, Tg, Tg/dt*k+1, k);
Ncon=[Ncon; T2vec(traj,1)];

traj=CartesianTrajectory(Tsc0*Tceg, Tsc0*Tcest, Ts, Ts/dt*k+1, k);
Ncon=[Ncon; T2vec(traj,1)];

traj=CartesianTrajectory(Tsc0*Tcest, Tsc1*Tcest, T2, Ts/dt*k+1, k);
Ncon=[Ncon; T2vec(traj,1)];

traj=CartesianTrajectory(Tsc1*Tcest, Tsc1*Tceg, Ts, Ts/dt*k+1, k);
Ncon=[Ncon; T2vec(traj,1)];

traj=CartesianTrajectory(Tsc1*Tceg, Tsc1*Tceg, Tg, Ts/dt*k+1, k);
Ncon=[Ncon; T2vec(traj,0)];

traj=CartesianTrajectory(Tsc1*Tceg, Tsc1*Tcest, Ts, Ts/dt*k+1, k);
Ncon=[Ncon; T2vec(traj,0)];

end


function [vec]=T2vec(M,s)
    vec=[];
    for i=1:size(M,2)
        T=M{i};
        temp=[];
        for j=1:3
            for m=1:3
                temp=[temp T(j,m)];
            end
        end
        for j=1:3
            temp=[temp T(j,4)];
        end
        temp=[temp s];
        vec=[vec;temp];
    end
end
end



