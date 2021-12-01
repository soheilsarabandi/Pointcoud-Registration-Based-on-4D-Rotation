function [T, time] = StandardMethod(A, B)
%
% This function returns the homogeneous transformation T that registers the 
% poincloud contained in A with respect to that in B using the SVD as 
% described in: 
%
% K.S. Arun, T.S. Huang, and S.D. Blostein, "Least-squares fitting of two 
% 3-D point sets," IEEE Trans. on Pattern Analysis and Machine Intelligence, 
% vol. 9, no. 5, pp. 698–700, 1987. 
%

tic;

[n,~] = size(B);

H = zeros(3,3);

a = sum(A)/n;
b = sum(B)/n;

H = B'*A - n*b'*a;

[U,~,V] = svd(H);

R = U*V';

T = [R, b' - R*a'; 0 0 0 1];

time = toc;
end
