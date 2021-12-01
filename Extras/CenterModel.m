function CenterModel(inputname,outputname)
%
% Example: CenterModel('bunny.ply', 'centeredbunny.ply')
%

ptCloudIn = pcread(inputname);

%pcshow(ptCloud);

A = ptCloudIn.Location;
 
A = A - mean(A);

ptCloudOut = pointCloud(A);
cmatrix = ones(size(ptCloudOut.Location)).*[1 0 0];
ptCloudOut = pointCloud(A,'Color',cmatrix);

pcshowpair(ptCloudIn, ptCloudOut, 'VerticalAxis','Y');

xlabel('X');
ylabel('Y');
zlabel('Z');

pcwrite(ptCloudOut, outputname);

end

