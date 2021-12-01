function PlotReciprocalModel(InputFile)
%
% This function takes as input a pointcloud, centers it and compute its 
% reciprocal pointcloud. Both pointclouds are scaled with respect to their 
% intrinsic scales. The resulting pointclouds are represented in separated 
% Windows. 
%
% Example: PlotReciprocalModel('..\Models\chair01.ply');
%

ptCloud = pcread(InputFile); 
A = double(ptCloud.Location);

AC = A-mean(A); % centered pointcloud
RE = AC*inv(AC'*AC); % reciprocal pointcloud

vol1 = det(cov(AC));
vol2 = det(cov(RE));

SA = vol1^(-1/6)*AC; % scaled original pointcloud
SR = vol2^(-1/6)*RE; % scaled reciprocal pointcloud

figure;
title(' Centered and Scaled Original Model');
hold on;
xlabel('x')
ylabel('y');
zlabel('z');
pcshow(pointCloud(SA),'VerticalAxis','Y');
%grid on;
%view(60,30);
ErrorEllipsoid(cov(SA))
hold off;

figure;
title(' Centered and Scaled Reciprocal Model');
hold on;
xlabel('x')
ylabel('y');
zlabel('z');
pcshow(pointCloud(SR),'VerticalAxis','Y');
%grid on;
%view(60,30);
ErrorEllipsoid(cov(SR))
hold off;

end
