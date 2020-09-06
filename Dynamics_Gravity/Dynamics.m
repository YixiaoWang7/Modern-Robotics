% dynamics with gravity
% See the dynamics of UR5
% It will generate csv files, which can be used in Vrep.

% parameters of UR5
M01 = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.089159; 0, 0, 0, 1];
M12 = [0, 0, 1, 0.28; 0, 1, 0, 0.13585; -1, 0, 0, 0; 0, 0, 0, 1];
M23 = [1, 0, 0, 0; 0, 1, 0, -0.1197; 0, 0, 1, 0.395; 0, 0, 0, 1];
M34 = [0, 0, 1, 0; 0, 1, 0, 0; -1, 0, 0, 0.14225; 0, 0, 0, 1];
M45 = [1, 0, 0, 0; 0, 1, 0, 0.093; 0, 0, 1, 0; 0, 0, 0, 1];
M56 = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.09465; 0, 0, 0, 1];
M67 = [1, 0, 0, 0; 0, 0, 1, 0.0823; 0, -1, 0, 0; 0, 0, 0, 1];
G1 = diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7]);
G2 = diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393]);
G3 = diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275]);
G4 = diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219]);
G5 = diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219]);
G6 = diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879]);
Glist = cat(3, G1, G2, G3, G4, G5, G6);
Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67); 
Slist = [0,         0,         0,         0,        0,        0;
         0,         1,         1,         1,        0,        1;
         1,         0,         0,         0,       -1,        0;
         0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491;
         0,         0,         0,         0,  0.81725,        0;
         0,         0,     0.425,   0.81725,        0,  0.81725];






%% situation 1
addpath('../lib/rob_yx')
thetalist=zeros(6,1);%initial thetalist
dthetalist=zeros(6,1);%initial velocity of thetalist
taulist=zeros(6,1);%initial acceleration of thetalist
g=[0;0;-9.81];%gravity
Ftip=[0;0;0;0;0;0];%no extern force applied on end effector

dt=0.005;%time step
t=0:dt:3;%time interval with the step of dt
thetalisthis=thetalist;%the time history of thetalist
for i=1:length(t)
    %forward dynamics
    ddthetalist = ForDynamics(thetalist,dthetalist,taulist,g,Ftip,Mlist,Glist,Slist);
    %integration
    thetalist=thetalist+dthetalist*dt;
    %record the thetalist
    thetalisthis=[thetalisthis thetalist];
    %integration
    dthetalist=dthetalist+ddthetalist*dt;
end
thetalisthis=thetalisthis';
csvwrite('s2_ForDynamics1_yx.csv',thetalisthis);


%% situation 2
addpath('../lib/rob_yx')
thetalist=zeros(6,1);%initial thetalist
thetalist(2)=-1;
dthetalist=zeros(6,1);%initial velocity of thetalist
taulist=zeros(6,1);%initial acceleration of thetalist
g=[0;0;-9.81];%gravity
Ftip=[0;0;0;0;0;0];%no extern force applied on end effector

dt=0.005;%time step
t=0:dt:5;%time interval with the step of dt
thetalisthis=thetalist;%the time history of thetalist
for i=1:length(t)
    %forward dynamics
    ddthetalist = ForDynamics(thetalist,dthetalist,taulist,g,Ftip,Mlist,Glist,Slist);
    %integration
    thetalist=thetalist+dthetalist*dt;
    %record the thetalist
    thetalisthis=[thetalisthis thetalist];
    %integration
    dthetalist=dthetalist+ddthetalist*dt;
end
thetalisthis=thetalisthis';
csvwrite('s2_ForDynamics2_yx.csv',thetalisthis);

