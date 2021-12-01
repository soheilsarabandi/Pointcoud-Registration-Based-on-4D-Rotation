function CharacteristicsPointCloud(InputFile)
%
% This function computes the shape characteristics of a given pointcloud. 
% 

ptCloud = pcread(InputFile); 

A = double(ptCloud.Location);
[n,~] = size(A);

AC = A-mean(A); % Centered pointcloud
RE = AC*inv(AC'*AC); % Reciprocal pointcloud

[V,D] = eig(cov(AC));

eigenvalues = diag(D);
eccentricity = sqrt(max(eigenvalues)/min(eigenvalues));

vol1 = sqrt(det(cov(AC)));
escale1= vol1^(1/3);

vol2 = sqrt(det(cov(RE)));
escale2= vol2^(1/3);

disp('Number of points');
disp(n);
disp('Centroid');
disp(a);
disp('Volume model');
disp(vol1);
disp('Intrinsic scale');
disp(escale1);
disp('Eigenvalues');
disp(eigenvalues);
disp('Eccentricity');
disp(eccentricity);
disp('Volume of the reciprocal');
disp(vol2);
disp('Intrinsic scale of the reciprocal');
disp(escale2);

end

