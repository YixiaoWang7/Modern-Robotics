function [V,Xe,TotalXe]=FeedbackControl(Tse, Tsed, Tsednext, Kp, Ki, dt, TotalXe)
addpath('../lib/mr')
Vd=1/dt*se3ToVec(MatrixLog6(TransInv(Tsed)*Tsednext));
Xe=se3ToVec(MatrixLog6(TransInv(Tse)*Tsed));
TotalXe=TotalXe+Xe*dt;
V=Adjoint(TransInv(Tse)*Tsed)*Vd+Kp*Xe+Ki*TotalXe;