function [theta0,success,thetahis]=IKinBodyIterates(J0,M,T_d,theta0,sigmaw,sigmav)
% Inverse kinematics in the body frame
% Expression:
%   [theta0,success,thetahis]=IKinBodyIterates(J0,M,T_d,theta0,sigmaw,sigmav)
% Input:
%   J0: body jaccobian matrix at zero position
%   M: transform matrix of body frame {b} expressed in base frame {s}
%   T_d: desired transform matrix of {b}
%   theta0: initial angle list of joints
%   sigmaw & sigmav: tolerance of the angular velocity and linear velocity
% Output:
%   theta0: final angle list
%   success: 0 or 1
%   thetahis: history of angle list over iterations

T0=ForKinB(J0,theta0,M);%forward kinematics in body frame
V_b=ISE3(ITransM(T0)*T_d);%SE3 to se3 then to twist
iter=0;%iteration
success=1;%whether to succeed
ew=norm(V_b(1:3));%angular error magnitude
ev=norm(V_b(4:6));%linear error magnitude
thetahis=theta0;%history of theta during iterations
while ew>sigmaw|ev>sigmav
    Jb=jacoRobB(J0,theta0);%body jacobian
    dtheta=pinv(Jb)*V_b;%dtheta
    theta0=theta0+dtheta;
    thetahis=[thetahis theta0];
    T0=ForKinB(J0,theta0,M);%forward kinematics in body frame
    V_b=ISE3(ITransM(T0)*T_d);%SE3 to se3 then to twist
    ew=norm(V_b(1:3));
    ev=norm(V_b(4:6));
    iter=iter+1;
    if iter>1000
        success=0;
    end
    %print the history
    format short
    fprintf(['Iteration ' num2str(iter) ':\n\n'])
    fprintf('joint vector:\n')
    for i=1:length(theta0)
        if i~=length(theta0)
            fprintf([num2str(theta0(i)) ', '])
        else
            fprintf([num2str(theta0(i))])
        end
    end
    fprintf('\n\n')
    fprintf('SE(3) end-effector config:\n')
    disp(T0)
    fprintf('error twist V_b:\n')
    for i=1:length(V_b)
        if i~=length(V_b)
            fprintf([num2str(V_b(i)) ', '])
        else
            fprintf([num2str(V_b(i))])
        end
    end
    fprintf('\n\n')
    fprintf(['angular error magnitude ||omega_b||: ' num2str(ew) '\n\n'])
    fprintf(['linear error magnitude ||v_b||: ' num2str(ev) '\n\n'])
end
thetahis=thetahis';