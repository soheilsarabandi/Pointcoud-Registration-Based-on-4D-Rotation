 function e = RandomS3()
% 
% This function returns points with a uniform distribution on the 
% surface of a 4-sphere following the algorithm described in:
%
% Marsaglia, G. "Choosing a Point from the Surface of a Sphere." 
% Ann. Math. Stat. 43, 645-646, 1972. 
%

while 1
    e(1) = 2*rand -1;
    e(2) = 2*rand -1;
    e(3) = 2*rand -1;
    e(4) = 2*rand -1;
    if (e(1)^2+e(2)^2)<1 && (e(3)^2+e(4)^2)<1
        norma = sqrt((1-e(1)^2-e(2)^2)/(e(3)^2+e(4)^2));
        e(3) = e(3)*norma;
        e(4) = e(4)*norma;
        % If, due numerical rounding errors, the norm is not exactly
        % equal to 1, the result is rejected
        if norm(e)==1
            break
        end
    end
end
end
