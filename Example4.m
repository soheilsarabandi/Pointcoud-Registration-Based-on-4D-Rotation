function Example4(File1,File2)
%
% This function takes two pointcloud models. Then, the correspondence between
% both point sets is obtained using the fives ICP methods contained in the 
% 'ICP Methods' subfolder. A table with the errors committed by the five 
% methods is displayed. The results of the five registrations are displayed
% in separated windows.
%
% Example: Example4('Bunny\bun045.ply','Bunny\bun000.ply');
%

% Initialization

clc; close all;

% Read the desired models 
ptCloud1 = (pcread(File1)); % magenta
ptCloud2 = pcread(File2); % green

P = double(ptCloud1.Location);
Q = double(ptCloud2.Location);

Tolerance=[0.0001, 0.0005]; %Tolerance


% Resgister both pointclouds using the five registration methods


[Transf1,~,Rmse(1), Time(1)]= pcregistericp_StandardMethod(ptCloud2,ptCloud1,'Verbose',true,'Tolerance',Tolerance, 'MaxIterations', 20);
Q1=(inv(Transf1.Rotation)*Q' + Transf1.Translation' )';

[Transf2,~,Rmse(2), Time(2)]= pcregistericp_QuaternionMethod(ptCloud2,ptCloud1,'Verbose',true,'Tolerance',Tolerance, 'MaxIterations', 20);
Q2=(inv(Transf2.Rotation)*Q' + Transf2.Translation' )';

[Transf3,~,Rmse(3), Time(3)]= pcregistericp_WuQuaternionMethod(ptCloud2,ptCloud1,'Verbose',true,'Tolerance',Tolerance,'MaxIterations', 20);
Q3=(inv(Transf3.Rotation)*Q' + Transf3.Translation' )';


[Transf4,~,Rmse(4), Time(4)]= pcregistericp_DualQuaternionMethod(ptCloud2,ptCloud1,'Verbose',true,'Tolerance',Tolerance, 'MaxIterations',20);
Q4=(inv(Transf4.Rotation)*Q' + Transf4.Translation' )';


[Transf5,~,Rmse(5), Time(5)]= pcregistericp_RefinedOurMethod(ptCloud2,ptCloud1,'Verbose',true,'Tolerance',Tolerance, 'MaxIterations', 20);
Q5=(inv(Transf5.Rotation)*Q' + Transf5.Translation' )';


[Transf6,~,Rmse(6), Time(6)]= pcregistericp_OurMethod(ptCloud2,ptCloud1,'Verbose',true,'Tolerance',Tolerance, 'MaxIterations', 20);
Q6=(inv(Transf6.Rotation)*Q' + Transf6.Translation' )';

% Display the registered pointclouds
figure;
hold on;
xlabel('x','Interpreter','Latex','FontWeight','bold','FontSize', 10)
ylabel('y','Interpreter','Latex','FontWeight','bold','FontSize', 10);
zlabel('z','Interpreter','Latex','FontWeight','bold','FontSize', 10);
pcshowpair(pointCloud(P), pointCloud(Q));
grid on;
axis equal;
set(gcf,'color','w');
set(gca,'color','w');
set(gca, 'XColor', 'k', 'YColor', 'k', 'ZColor','k');
view(170, 35);
camroll(180);
title('Initialization','Interpreter','Latex','FontWeight','bold','FontSize', 10,'color','k');
hold off;


figure;
hold on;
xlabel('x','Interpreter','Latex','FontWeight','bold','FontSize', 10)
ylabel('y','Interpreter','Latex','FontWeight','bold','FontSize', 10);
zlabel('z','Interpreter','Latex','FontWeight','bold','FontSize', 10);
pcshowpair(pointCloud(P), pointCloud(Q1));
grid on;
set(gcf,'color','w');
set(gca,'color','w');
set(gca, 'XColor', 'k', 'YColor', 'k', 'ZColor','k');
view(170, 35);
camroll(180);
title('Standard Method','Interpreter','Latex','FontWeight','bold','FontSize', 10,'color','k');
hold off;

figure;
hold on;
xlabel('x','Interpreter','Latex','FontWeight','bold','FontSize', 10)
ylabel('y','Interpreter','Latex','FontWeight','bold','FontSize', 10);
zlabel('z','Interpreter','Latex','FontWeight','bold','FontSize', 10);
pcshowpair(pointCloud(P), pointCloud(Q2));
grid on;
set(gcf,'color','w');
set(gca,'color','w');
set(gca, 'XColor', 'k', 'YColor', 'k', 'ZColor','k');
view(170, 35);
camroll(180);
title('Quaternion method','Interpreter','Latex','FontWeight','bold','FontSize', 10,'color','k');
hold off;

figure;
hold on;
xlabel('x','Interpreter','Latex','FontWeight','bold','FontSize', 10)
ylabel('y','Interpreter','Latex','FontWeight','bold','FontSize', 10);
zlabel('z','Interpreter','Latex','FontWeight','bold','FontSize', 10);
pcshowpair(pointCloud(P), pointCloud(Q3));
grid on;
set(gcf,'color','w');
set(gca,'color','w');
set(gca, 'XColor', 'k', 'YColor', 'k', 'ZColor','k');
view(170, 35);
camroll(180);
title('Wu Quaternion method','Interpreter','Latex','FontWeight','bold','FontSize', 10,'color','k');
hold off;


figure;
hold on;
xlabel('x','Interpreter','Latex','FontWeight','bold','FontSize', 10)
ylabel('y','Interpreter','Latex','FontWeight','bold','FontSize', 10);
zlabel('z','Interpreter','Latex','FontWeight','bold','FontSize', 10);
pcshowpair(pointCloud(P), pointCloud(Q4));
grid on;
set(gcf,'color','w');
set(gca,'color','w');
set(gca, 'XColor', 'k', 'YColor', 'k', 'ZColor','k');
view(170, 35);
camroll(180);
title('Dual quaternion method','Interpreter','Latex','FontWeight','bold','FontSize', 10,'color','k');
hold off;


figure;
hold on;
xlabel('x','Interpreter','Latex','FontWeight','bold','FontSize', 10)
ylabel('y','Interpreter','Latex','FontWeight','bold','FontSize', 10);
zlabel('z','Interpreter','Latex','FontWeight','bold','FontSize', 10);
pcshowpair(pointCloud(P), pointCloud(Q5));
grid on;
set(gcf,'color','w');
set(gca,'color','w');
set(gca, 'XColor', 'k', 'YColor', 'k', 'ZColor','k');
view(170, 35);
camroll(180);
title('Our Method','Interpreter','Latex','FontWeight','bold','FontSize', 10,'color','k');
hold off;

% Present a table compiling all errors
T1 = table({'Time';'RMS Error';},[Time(1) ;Rmse(1)],[Time(2); Rmse(2)],[Time(3) ;Rmse(3)],[Time(4); Rmse(4)],[Time(5) ;Rmse(5)],[Time(6) ;Rmse(6)]);
T1.Properties.VariableNames = {'Result','Standard Method','Quaternion method','Wu Quaternion method',...
'Dual quaternion method','Refined4D rotation','4D rotation'}