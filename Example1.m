function Example1(File)
%
% This function takes a pointcloud model, applies a given homogeneous 
% transformation to it, and adds some noise to some randomly chosen points. 
% Then, the original poincloud is registered with respect to the displaced 
% one using the fives methods contained in the 'Methods' subfolder. 
% A table with the errors committed by the five methods is displayed. 
% The results of the five registrations are displayed in separated windows.
%
% Example: Example1('Models\bunny.ply');
%

% Initialization

clc; close all;

% Generate an arbitrary homogeneous transformation

R = rotx(pi/3, 'rad')*roty(pi/6, 'rad')*rotz(pi/4, 'rad'); 
e = Cayley(R);
t = [0.2;0.5;0.1];
Transf = ([R t; 0 0 0 1]);

% Read the desired model 

ptCloud = pcread(File);

A = double(ptCloud.Location);
[n,~] = size(A);

% Apply the transformation to the model

B1 = Transf*[A'; ones(1,n)];
B  = B1(1:3,:)';

% Randomly choose 80 percent of the point indices

l = 80*round(n/100);
perm = randperm(n);
out = perm(1:l);

% Add noise with Gaussian distribution to 80 percent of the points according 
% to the following Gaussian distributions:
% (     0, 0.020) to 50 percent of the points, 
% ( 0.005, 0.018) to 25 percent of the points, and 
% (-0.005, 0.018) to 25 percent of the points.

for i=1:4:l
    B(out(i),  :)= B(out(i),  :) + 0.02*randn(1,3);
    B(out(i+1),:)= B(out(i+1),:) + 0.02*randn(1,3);
    B(out(i+2),:)= B(out(i+2),:) + 0.0018*randn(1,3) + [ 0.005  0.005  0.005];
    B(out(i+3),:)= B(out(i+3),:) + 0.0018*randn(1,3) + [-0.005 -0.005 -0.005];
end

% Resgister the reference pointcloud with respect to the displaced pointcloud
% using the five registration methods

[Transf1, Time(1)] = StandardMethod(A, B);
[Transf2, Time(2)] = QuaternionMethod(A, B);
[Transf3, Time(3)] = WuQuaternionMethod(A, B);
[Transf4, Time(4)] = DualQuaternionMethod(A, B);
[Transf5, Time(5)] = OurMethod(A, B);

% Compute the rotational errors

Rerror(1) = acos(abs(dot(e,Cayley(Transf1(1:3,1:3)))));
Rerror(2) = acos(abs(dot(e,Cayley(Transf2(1:3,1:3)))));
Rerror(3) = acos(abs(dot(e,Cayley(Transf3(1:3,1:3)))));
Rerror(4) = acos(abs(dot(e,Cayley(Transf4(1:3,1:3)))));
Rerror(5) = acos(abs(dot(e,Cayley(Transf5(1:3,1:3)))));

% Compute the translational error

Terror(1) = norm(Transf(1:3,4)-Transf1(1:3,4));
Terror(2) = norm(Transf(1:3,4)-Transf2(1:3,4));
Terror(3) = norm(Transf(1:3,4)-Transf3(1:3,4));
Terror(4) = norm(Transf(1:3,4)-Transf4(1:3,4));
Terror(5) = norm(Transf(1:3,4)-Transf5(1:3,4));

% Compute the orthogonality error of our method

orthe = abs(1-det(Transf5(1:3,1:3)))

% Present a table compiling all errors

Method = {'Standard Method';'Quaternion method';'Wu Quaternion method';...
          'Dual quaternion method';'Our Method'};
Table = table(Method, 1000*Rerror', 1000*Terror', 1000*Time');
Table.Properties.VariableNames = {'Method', 'RotationalError', ...
                                  'TranslationalError', 'Time'}

% Display the registered pointclouds

B1 = Transf1*[A'; ones(1,n)];
B2 = Transf2*[A'; ones(1,n)];
B3 = Transf3*[A'; ones(1,n)];
B4 = Transf4*[A'; ones(1,n)];
B5 = Transf5*[A'; ones(1,n)];

figure;
%pcshowpair(pointCloud(A), pointCloud(B), 'VerticalAxis','Y');
hold on;
trplot(eye(3),'arrow','length',0.3, 'width', 0.5);
PlotCube(max(A)-min(A), min(A), 0.1, [0.5 0 0]);
PlotCube(max(B)-min(B), min(B), 0.1, [0 0.5 0]);
pcshowpair(pointCloud(A), pointCloud(B), 'VerticalAxis','Y');
set(gcf,'color','w');
set(gca,'color','w');
set(gca, 'XColor', 'k', 'YColor', 'k', 'ZColor','k');
title('Initialization','Interpreter','Latex','FontWeight','bold','FontSize', 10,'color','k');
grid on;
hold off;

figure;
hold on;
xlabel('x')
ylabel('y');
zlabel('z');
pcshowpair(pointCloud(B1(1:3,:)'), pointCloud(B), 'VerticalAxis','Y');
set(gcf,'color','w');
set(gca,'color','w');
set(gca, 'XColor', 'k', 'YColor', 'k', 'ZColor','k');
title('Standard Method','Interpreter','Latex','FontWeight','bold','FontSize', 10,'color','k');
grid on;
hold off;

figure;
hold on;
xlabel('x')
ylabel('y');
zlabel('z');
pcshowpair(pointCloud(B2(1:3,:)'), pointCloud(B), 'VerticalAxis','Y');
set(gcf,'color','w');
set(gca,'color','w');
set(gca, 'XColor', 'k', 'YColor', 'k', 'ZColor','k');
title('Quaternion method','Interpreter','Latex','FontWeight','bold','FontSize', 10,'color','k');
grid on;
hold off;

figure;
hold on;
xlabel('x')
ylabel('y');
zlabel('z');
pcshowpair(pointCloud(B3(1:3,:)'), pointCloud(B), 'VerticalAxis','Y');
set(gcf,'color','w');
set(gca,'color','w');
set(gca, 'XColor', 'k', 'YColor', 'k', 'ZColor','k');
title('Wu Quaternion method','Interpreter','Latex','FontWeight','bold','FontSize', 10,'color','k');
grid on;
hold off;

figure;
hold on;
xlabel('x')
ylabel('y');
zlabel('z');
pcshowpair(pointCloud(B4(1:3,:)'), pointCloud(B), 'VerticalAxis','Y');
set(gcf,'color','w');
set(gca,'color','w');
set(gca, 'XColor', 'k', 'YColor', 'k', 'ZColor','k');
title('Dual quaternion method','Interpreter','Latex','FontWeight','bold','FontSize', 10,'color','k');
grid on;
hold off;

figure;
hold on;
xlabel('x')
ylabel('y');
zlabel('z');
pcshowpair(pointCloud(B5(1:3,:)'), pointCloud(B), 'VerticalAxis','Y');
set(gcf,'color','w');
set(gca,'color','w');
set(gca, 'XColor', 'k', 'YColor', 'k', 'ZColor','k');
title('Our Method','Interpreter','Latex','FontWeight','bold','FontSize', 10,'color','k');
grid on;
hold off;

end
