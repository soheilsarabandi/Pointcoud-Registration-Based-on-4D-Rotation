function PlotEllipsoid(C)
%
% This function takes as input a 3x3 covariance matrix, and plots the
% associated ellipsoid.
%

[eigvec,eigval] = eig(C);

if any(diag(eigval)<=0)
    error('The covariance matrix must be positive definite');
end

[X,Y,Z] = ellipsoid(0,0,0,1,1,1,40);
M = sqrt(eigval)*eigvec';
XYZ = [X(:),Y(:),Z(:)]*M;
  
X(:) = XYZ(:,1);
Y(:) = XYZ(:,2);
Z(:) = XYZ(:,3);
  
surf(X,Y,Z, 'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', 'interp'); 
camlight('right');
shading interp; 
light;               
lighting gouraud;

arrow3([0 0 0], M(1,:));
arrow3([0 0 0], -M(1,:));
arrow3([0 0 0], M(2,:));
arrow3([0 0 0], -M(2,:));
arrow3([0 0 0], M(3,:));
arrow3([0 0 0], -M(3,:));

end
 