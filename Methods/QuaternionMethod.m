function [T,time] = QuaternionMethod(A, B)
%
% This function returns the homogeneous transformation T that registers the 
% poincloud contained in A with respect to that in B using the method based 
% on quaternions described in: 
%
% B.K.P. Horn, "Closed-form solution of absolute orientation using unit 
% quaternions," Journal of the Optical Society of America A, vol. 4, no. 4,
% pp. 629-642, 1987.
%

tic;
 
[n,~] = size(B);
 
a = sum(A)/n;
b = sum(B)/n;

H = B'*A - n*b'*a;

G = [H(1,1)+H(2,2)+H(3,3),-H(2,3)+H(3,2),       -H(3,1)+H(1,3),       -H(1,2)+H(2,1);
    -H(2,3)+H(3,2),        H(1,1)-H(2,2)-H(3,3), H(1,2)+H(2,1),        H(1,3)+H(3,1);
    -H(3,1)+H(1,3),        H(1,2)+H(2,1),        H(2,2)-H(1,1)-H(3,3), H(2,3)+H(3,2);
    -H(1,2)+H(2,1),        H(1,3)+H(3,1),        H(2,3)+H(3,2),        H(3,3)-H(2,2)-H(1, 1)];

[V, D] = eig(G);

[~,index]=max(diag(D));

R = Quat2Mat(V(:,index));

T = [R, b' - R*a'; 0 0 0 1];

time = toc;

 end