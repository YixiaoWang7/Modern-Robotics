function [nextcon]=NextStage(currentcon,controls,dt,limits)
%currentcon: A 12by1 vector representing the current configuration 
%of the robot (3 variables for the chassis configuration, 5 variables 
%for the arm configuration, and 4 variables for the wheel angles)

%controls:A 9-vector of controls indicating the velocity

%dt:time interval

%limits:maximum velocity.separate


for i=1:9
    if controls(i)<0
        if controls(i)<-limits(i)
            controls(i)=-limits(i);
        end
    else
        if controls(i)>limits(i)
            controls(i)=limits(i);
        end
    end
end
nextcon=zeros(12,1);
nextcon(4:8)=currentcon(4:8)+controls(1:5)*dt;
nextcon(9:12)=currentcon(9:12)+controls(6:9)*dt;
l=0.47/2;
r=0.0475;
w=0.3/2;
vb=r/4*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w);1 1 1 1;-1 1 -1 1]*dt*controls(6:9);
angle=currentcon(1);
R=[1 0 0 ; 0 cos(angle) -sin(angle);0 sin(angle) cos(angle)];
if vb(1)==0
    nextcon(1:3)=currentcon(1:3)+R*vb;
else
    nextcon(1:3)=currentcon(1:3)+R*[vb(1) (vb(2)*sin(vb(1))+vb(3)*(cos(vb(1))-1))/vb(1) (vb(3)*sin(vb(1))+vb(2)*(-cos(vb(1))+1))/vb(1)]';
end



