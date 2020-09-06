function [A]=adv(vi)
% Calculate adv in frame {i}
% Expression:
%   [A]=adv(vi)
% Input:
%   vi: twist in frame {i}
% Output:
%   A: adv matrix

A=zeros(6,6);
A(1:3,1:3)=CrossM(vi(1:3));
A(4:6,4:6)=CrossM(vi(1:3));
A(4:6,1:3)=CrossM(vi(4:6));
