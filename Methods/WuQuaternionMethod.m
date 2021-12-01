function [T,time] = WuQuaternionMethod(A, B)
%
% This function returns the homogeneous transformation T that registers the 
% poincloud contained in A with respect to that in B using the method based 
% on a quaternion representation described in: 
%
% J. Wu, M. Liu, Z. Zhou, and R. Li, "Fast symbolic {3D} registration 
% solution," IEEE Transactions on Automation Science and Engineering, to
% appear.
% 
% This function is an improved version of Jin Wu's own implementation.
%
tic;
 
[n,~] = size(B);
 
t1 = sum(A)/n;
t2 = sum(B)/n;

H = B'*A - n*t2'*t1;

G = [H(1,1)+H(2,2)+H(3,3),-H(2,3)+H(3,2),       -H(3,1)+H(1,3),       -H(1,2)+H(2,1);
    -H(2,3)+H(3,2),        H(1,1)-H(2,2)-H(3,3), H(1,2)+H(2,1),        H(1,3)+H(3,1);
    -H(3,1)+H(1,3),        H(1,2)+H(2,1),        H(2,2)-H(1,1)-H(3,3), H(2,3)+H(3,2);
    -H(1,2)+H(2,1),        H(1,3)+H(3,1),        H(2,3)+H(3,2),        H(3,3)-H(2,2)-H(1, 1)];

c = det(G);
b = -8*det(H);
a = -2*sum(H.^2, 'all');

p0 = 2*a*a*a + 27*b*b - 72*a*c;

theta = atan2(sqrt(abs(4*(a*a + 12*c)^3 - p0*p0)), p0);

p1 = 2*sqrt(sqrt(a*a + 12*c)*cos(theta/3) - a);

if abs(b)<1e-5 && abs(p1)<1e-5
    lambda = sqrt(-b/2);
else
    lambda = 0.204124145231932*(p1 + sqrt(-p1*p1 - 12*a - 29.393876913398135*b/p1));
end

F = G - lambda*eye(4);
J = adjoint(F);
[~, index] = max(sum(J.^2));
R = Quat2Mat(J(:,index));

T = [R, t2' - R*t1'; 0 0 0 1];

time = toc;

 end