%project
%type 'runscript' in the Command Window and press 'enter', the program will
%automatically run. you will see
%"
%<<runscript
%Generating animation csv file.
%Writing error and plot data.
%Done.
%"
%to check whether this program works well. The program will produce two
%.csv files and plot one Xe figure.
%The program is divided into several sections: user define, predefined
%physical known data, generate trajectory and control.
%You can modify the initial position of the cube and so on.
%joint limits to avoid collisions are commented.
%
%For the best result, do not need to change anything just run.
%For the overshoot result, change Ki and Kp into Kp=2*eye(6);Ki=5*eye(6).
%For the newTask result, change intial and target position of the cube into
%cube0=[1 0.5 pi/4];cube1=[0.2 -1.2 -3*pi/4];
%For the collision_intial where collision occurs, change intial position of
%the cube into cube0=[-1 1 pi/4]; and change the control iteration times
%into 1500 at around 97th row.
%For the collision_fail, use the part collision limits beginning with 'For 
%collison' and ended with 'collision end' at around 129th~134th rows;
%For the collision_sucess, change the Tseg and Tsest into Tceg=[-1 0 0 0;0
%1 0 0 ;0 0 -1 0;0 0 0 1];Tcest=[-1 0 0 0;0 1 0 0 ;0 0 -1 0.08;0 0 0 1];


%% user define
%initial position of the cube
cube0=[1 0 0];
Tsc0=[cos(cube0(3)) -sin(cube0(3)) 0 cube0(1);sin(cube0(3)) cos(cube0(3)) 0 cube0(2) ;0 0 1 0.025;0 0 0 1];
%target position of the cube
cube1=[0 -1 -pi/2];
Tsc1=[cos(cube1(3)) -sin(cube1(3)) 0 cube1(1);sin(cube1(3)) cos(cube1(3)) 0 cube1(2) ;0 0 1 0.025;0 0 0 1];
%initial position of the end-effector
Tse0=[0 0 1 0;0 1 0 0; -1 0 0 0.5;0 0 0 1];
%end-effector's configuration relative to the cube when grasping the cube
Tceg=[-1/2 0 sqrt(3)/2 0.008;0 1 0 0;-sqrt(3)/2 0 -1/2 0.00;0 0 0 1];
%end-effector's standoff configuration
Tcest=[-1/2 0 sqrt(3)/2 0.008;0 1 0 0 ;-sqrt(3)/2 0 -1/2 0.08;0 0 0 1];
%multiple coefficient
k=1;
%PI control
Kp=5*eye(6);
Ki=0.01*eye(6);
%time step
dt=0.01;
%current configuration
currentcon=[-0.2 -1 0 0 0 -2 0.1 0 0 0 0 0]';


%% predefined physical known data
%add the library. I am worried about whether my pogram works if I move the
%library here, so I reserve this code and the library.
addpath('../lib/mr')

%Predefined. Physical known data.
%Fixed offset from the chassis frame {b} to the base frame of the arm {0}
Tbo=[1 0 0 0.1662;0 1 0 0;0 0 1 0.0026;0 0 0 1];
%the end-effector frame {e} relative to the arm base frame {0}
Moe=[1 0 0 0.033;0 1 0 0 ;0 0 1 0.6546;0 0 0 1];
%the screw axes for the five joints expressed in the end-effector frame {e}
Blist=[0 0 0 0 0;0 -1 -1 -1 0;1 0 0 0 1;0 -0.5076 -0.3526 -0.2176 0;0.0330 0 0 0 0;0 0 0 0 0];

%paw parameter. Just for the grasping and standoff configurations 
d2=0.035;
d3=0.043;
s=0.05;

%mnidirectional mobile base
l=0.47/2;
r=0.0475;
w=0.3/2;
H=r/4*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w);1 1 1 1;-1 1 -1 1];
H=[zeros(2,4) ;H; zeros(1,4)];


%% generate trajectory
[Tdset]=TrajectoryGenerator(Tse0, Tsc0, Tsc1, Tceg, Tcest, k, 0);
%write the trajectory
%csvwrite('s8_gripper.csv',Tdset)


%% control
%real configuration adding grasping condition 0/1.
rcon=[currentcon;0]';
%Set of Xes in every time step.
XeSet=[];
%Total Xe
TotalXe=zeros(6,1);
%joint velocity limits
limits=100*ones(1,9);
%control
for i=1:size(Tdset,1)-1
% for i=1:1500
    %renew the configuration of end-effector
    Tsb=[cos(currentcon(1)) -sin(currentcon(1)) 0 currentcon(2); sin(currentcon(1)) cos(currentcon(1)) 0 currentcon(3); 0 0 1 0.0963;0 0 0 1];
    Toe=FKinBody(Moe, Blist, currentcon(4:8));
    Tse=Tsb*Tbo*Toe;
    %generate the transform matrix
    Tsed=eye(4);
    Tsednext=eye(4);
    for j=1:3
        for k=1:3
            Tsed(j,k)=Tdset(i,(j-1)*3+k);
            Tsednext(j,k)=Tdset(i+1,(j-1)*3+k);
        end
    end
    for j=1:3
        Tsed(j,4)=Tdset(i,j+9);
        Tsednext(j,4)=Tdset(i+1,j+9);
    end
    %feedback control
    [V,Xe,TotalXe]=FeedbackControl(Tse, Tsed, Tsednext, Kp, Ki, dt, TotalXe);
    %record the Xe
    XeSet=[XeSet Xe];
    %generate the control signal or velocity
    Ju=Adjoint(TransInv(Tbo*Toe))*H;
    Jb=JacobianBody(Blist, currentcon(4:8));
    Je=[Ju Jb];
    controls=pinv(Je,1e-2)*V;
    %note that velocity should be one to one
    controls=[controls(5:9);controls(1:4)];
    %next stage. renew the configuration
    [nextcon]=NextStage(currentcon,controls,dt,limits);
    
%     %For collision
%     [wrong]=testjointlimits(nextcon);
%     if ~isempty(wrong)
%         Je(:,4+wrong)=zeros(6,length(wrong));
%         controls=pinv(Je,1e-3)*V;
%         controls=[controls(5:9);controls(1:4)];
%         [nextcon]=NextStage(currentcon,controls,dt,limits);
%     end
%     %collision end

    currentcon=nextcon;
    %add the configuration into rcon
    rcon=[rcon;[currentcon;Tdset(i,end)]'];
end
disp('Generating animation csv file.')
csvwrite('s6_Vrep_best.csv',rcon)
disp('Writing error and plot data.')
csvwrite('s6_XeSet_best.csv',XeSet)

n=size(XeSet,2);
for i=1:6
    plot(0:dt:dt*(n-1),XeSet(i,:))
    hold on
end
legend('e_x','e_y','e_z','e_{Wx}','e_{Wy}','e_{Wz}')
xlabel('t/s')
ylabel('error')
disp('Done.')




