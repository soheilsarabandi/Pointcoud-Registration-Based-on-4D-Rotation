function [T,time] = DualQuaternionMethod(A, B)
%
% This function returns the homogeneous transformation T that registers the 
% poincloud contained in A with respect to that in B using the method based 
% on dual quaternions described in: 
%
% M.W. Walker, L. Shao, and R.A. Volz, "Estimating 3-D location parameters 
% using dual number quaternions," CVGIP: Image Understanding, vol. 54,
% no. 3, pp. 358-367, 1991.
%
tic;
[n, ~] = size(A);

C1 = zeros(4, 4);
C2 = n*eye(4); 
C3 = zeros(4, 4);
T  = zeros(4, 4);

for i=1:n
  Q = [0      -B(i,1) -B(i,2) -B(i,3);
       B(i,1)  0      -B(i,3)  B(i,2);
       B(i,2)  B(i,3)  0      -B(i,1);
       B(i,3) -B(i,2)  B(i,1)  0];
  W = [0      -A(i,1) -A(i,2) -A(i,3);
       A(i,1)  0       A(i,3) -A(i,2);
       A(i,2) -A(i,3)  0       A(i,1);
       A(i,3)  A(i,2) -A(i,1)  0];
  C1 = C1 + 1/2*Q*W;      
  C3 = C3 + W - Q;
end

A = 1/(2*n)*C3'*C3 - C1 - C1';
 
[V,D] = eig(A);
[~,index]= max(diag(D));
 
e = V(:,index);

t = (-1/n)* ...
    [-e(2)  e(1) -e(4)  e(3);
     -e(3)  e(4)  e(1) -e(2);
     -e(4) -e(3)  e(2)  e(1)]*C3*e;

R = Quat2Mat(e);

T = [R, t; 0 0 0 1];

time = toc;

end