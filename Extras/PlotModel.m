function PlotModel(InputFile)
%
% This function plot a given pointcloud
%
% Example: PlotModel('Models\chair00.ply');
%

ptCloud = pcread(InputFile); 

A = double(ptCloud.Location);

figure;
title(InputFile);
hold on;
xlabel('x')
ylabel('y');
zlabel('z');
%pcshowpair(pointCloud(A),pointCloud(B),'VerticalAxis','Y');
pcshow(pointCloud(A),'VerticalAxis','Y');
%grid on;
%view(60,30);
hold off;

end

