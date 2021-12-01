function e = RandomS2()
% 
% This function returns points with a uniform distribution on the 
% surface of a 3-sphere following the algorithm described in:
%
% Marsaglia, G. "Choosing a Point from the Surface of a Sphere." 
% Ann. Math. Stat. 43, 645-646, 1972. 
%

while 1
    v1 = 2*rand -1;
    v2 = 2*rand -1;
    s = v1^2 + v2^2;
    if s < 1
        e = [2*v1*sqrt(1-s), 2*v2*sqrt(1-s), 1-2*s];
        break;
    end
end
end

