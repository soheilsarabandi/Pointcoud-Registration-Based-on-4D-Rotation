function Example2(File)
%
% This function takes a pointcloud model, applies a given homogeneous 
% transformation to it, and adds some noise to some randomly chosen points. 
% Them, the original poincloud is registered with respect to the displaced 
% one using Wu et al's and our method. This process is repeated 10^4 
% randomly generated homogeneous transformations to obtain the error 
% statistic for the two methods. 
% 
% Example: Example2('Models\bunny.ply');
%

% Initialization

clc; close all;

% Read the desired model

ptCloud = pcread(File);

A = ptCloud.Location;
A = double(ptCloud.Location);

[n,~] = size(A);

for k=1:10000
    
    % Generate a random homogeneous transformation
    e = RandomS3;
    R = Quat2Mat(e);
    t = 10.0*RandomS2';
    Transf = ([R t; 0 0 0 1]);
    
    % Apply the transformation to the model
    B1 = Transf*[A'; ones(1,n)];
    B  = B1(1:3,:)';
    
    % Randomly choose 80 percent of the point indices
    l = 80*round(n/100);
    perm = randperm(n);
    out = perm(1:l);
    
    % Add noise with Gaussian distribution to 80 percent of the point according
    % to the following Gaussian distributions:
    % (     0, 0.020) to 50 percent of the points,
    % ( 0.005, 0.018) to 25 percent of the points, and
    % (-0.005, 0.018) to 25 percent of the points.
    
    for j=1:4:l
        B(out(j),  :) = B(out(j),  :) + 0.02*randn(1,3);
        B(out(j+1),:) = B(out(j+1),:) + 0.02*randn(1,3);
        B(out(j+2),:) = B(out(j+2),:) + 0.0018*randn(1,3) + [ 0.005  0.005  0.005];
        B(out(j+3),:) = B(out(j+3),:) + 0.0018*randn(1,3) + [-0.005 -0.005 -0.005];
    end
    
    % Resgister the reference pointcloud with respect the displaced one 
    % using Wu et al's method and our method.
    [Transf1, Time(k,1)] = WuQuaternionMethod(A, B);
    [Transf2, Time(k,2)] = OurMethod(A, B);
    
    % Compute the rotational errors
    Rerror(k,1) = acos(min(1,abs(dot(e,Cayley(Transf1(1:3,1:3))))));
    Rerror(k,2) = acos(min(1,abs(dot(e,Cayley(Transf2(1:3,1:3)))))); 
    
    % Compute the translational errors
    Terror(k,1) = norm(Transf(1:3,4) - Transf1(1:3,4));
    Terror(k,2) = norm(Transf(1:3,4) - Transf2(1:3,4));
    
    % Compute the orthogonality error for our method
    Orth(k) = abs(1-det(Transf2(1:3,1:3)));

end

% Set some scale factors for the errors and format them in order to be 
% presentaed in a table

sc1 = 10^3;
sc2 = 10^6;

Rerror1 = [sc1*max(Rerror(:,1)), sc1*mean(Rerror(:,1)), sc1*min(Rerror(:,1)), sc2*var(Rerror(:,1))];
Rerror2 = [sc1*max(Rerror(:,2)), sc1*mean(Rerror(:,2)), sc1*min(Rerror(:,2)), sc2*var(Rerror(:,2))];

Terror1 = [sc1*max(Terror(:,1)), sc1*mean(Terror(:,1)), sc1*min(Terror(:,1)), sc2*var(Terror(:,1))];
Terror2 = [sc1*max(Terror(:,2)), sc1*mean(Terror(:,2)), sc1*min(Terror(:,2)), sc2*var(Terror(:,2))];

time1 = mean(Time(:,1));
time2 = mean(Time(:,2));

Orthe = [max(Orth), mean(Orth), min(Orth), var(Orth)];

% Presentation of the error tables

T1 = table({'Maximum';'Mean';'Minimum';'Variance'}, Rerror1', Rerror2');
T1.Properties.VariableNames = {'RotationalError', 'WuMethod', 'OurMethod'}

T2 = table({'Maximum';'Mean';'Minimum';'Variance'}, Terror1', Terror2');
T2.Properties.VariableNames = {'TranslationalError', 'WuMethod', 'OurMethod'}

T3 = table({10^3*time1, 10^3*time2});
T3.Properties.VariableNames = {'ComputationalCost'}

T4 = table(Orthe)

end