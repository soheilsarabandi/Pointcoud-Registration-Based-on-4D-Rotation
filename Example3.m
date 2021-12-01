function Example3(File)
%
% This function takes as input a pointcloud model, applies a given 
% homogeneous transformation to it, and adds a variable level of noise. 
%
% Example: Example3('Models\chair00.ply');
%

% Initialization

clc; close all;

% Generate an arbitrary homogeneous transformation

R = rotx(pi/3, 'rad')*roty(pi/6, 'rad')*rotz(pi/4, 'rad'); 
e = Cayley(R);
t = [0.2;0.5;0.8];
T = ([R t; 0 0 0 1]);

% Read the desired model 

ptCloud = pcread(File);

A = double(ptCloud.Location);
[n,~] = size(A);

% Apply the transformation to the model

B1 = T*[A'; ones(1,n)];
B2  = B1(1:3,:)';

samples = 200;

sigma2 = linspace(0,0.01,samples);

for i=1:10
   for j=1:samples
    % Add zero mean noise with Gaussian distribution to all points
    B =  B2 + sigma2(j)*randn(n,3);
    
    % Resgister the reference pointcloud with respect to the displaced pointcloud
    % using the proposed method
    [Te, time] = OurMethod(A, B);
    
    % Compute the rotational error
    Rerror(i,j) = real(acos(abs(dot(e,Cayley(Te(1:3,1:3))))));
    
    % Compute the translational error
    Terror(i,j) = norm(T(1:3,4)-Te(1:3,4));
    
    % Compute the orthogonality error of the proposed method
    Orthe(i,j) = abs(1-det(Te(1:3,1:3)));
   end
end

mRerror = mean(Rerror);
mTerror = mean(Terror);
mOrthe  = mean(Orthe);

coeR = polyfit(sigma2,mRerror,1);
coeT = polyfit(sigma2,mTerror,1);
coeO = polyfit(sigma2,mOrthe,1);

disp(coeR(1));
disp(coeT(1));
disp(coeO(1));

figure(1);
plot(sigma2, mRerror);
hold on;
xlabel('sigma2');
ylabel('Rotational error');
plot([sigma2(1) sigma2(samples)], [mRerror(1) coeR(1)*sigma2(samples)]);
legend('',num2str(coeR(1)));
grid on;
 
figure(2);
plot(sigma2, mTerror);
hold on;
xlabel('sigma2');
ylabel('Translational error');
plot([sigma2(1) sigma2(samples)], [mTerror(1) coeT(1)*sigma2(samples)]);
legend('',num2str(coeT(1)));
grid on;

figure(3);
plot(sigma2, mOrthe);
hold on;
xlabel('sigma2');
ylabel('Orthogonality error');
plot([sigma2(1) sigma2(samples)], [mOrthe(1) coeO(1)*sigma2(samples)]);
legend('',num2str(coeO(1)));
grid on;

end

