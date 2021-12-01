function [T, time] = Our_RefinedMethod(A, B)
%
% This function returns the homogeneous transformation T that registers the 
% poincloud contained in A with respect to that in B using the method 
% described in: 
%
% S. Sarabandi and F. Thomas, "Approximating displacements in R^3 by 
% rotations in R^4 and its application to pointcloud registation,"
% Submitted to IEEE Transaction on Pattern Analysis and Machine
% Intelligence, 2020.
%

tic;

[n, ~] = size(A);

a = sum(A)/n;
b = sum(B)/n;

K = inv(A'*A - n*a'*a);
H = B'*A - n*b'*a;
R = H*K;

M = R'*R;
R = R*((3*eye(3))+M)*((eye(3)+(3*M))^-1); % Cubically convergent dual


T = [R, b' - R*a'; 0 0 0 1];

time = toc;

 end