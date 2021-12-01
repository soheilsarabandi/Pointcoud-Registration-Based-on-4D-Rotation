function e = Cayley(R)
%
% This function takes as input a 3x3 rotation matrix and returns the
% corresponding unit quaternion. It implements Cayley's method. For 
% details, see:
%
% S. Sarabandi and F. Thomas, "A survey on the computation of quaternions 
% from rotation matrices," ASME Journal of Mechanisms and Robotics, 
% Vol. 11, No. 2, 021006, 2019
%

e(1) = 0.25*sqrt((1+R(1,1)+R(2,2)+R(3,3))^2 + (R(3,2)-R(2,3))^2 + (R(1,3)-R(3,1))^2 + (R(2,1)-R(1,2))^2);
e(2) = 0.25*sqrt((R(3,2)-R(2,3))^2 + (1+R(1,1)-R(2,2)-R(3,3))^2 + (R(1,2)+R(2,1))^2 + (R(3,1)+R(1,3))^2);
e(3) = 0.25*sqrt((R(1,3)-R(3,1))^2 + (R(1,2)+R(2,1))^2 + (1-R(1,1)+R(2,2)-R(3,3))^2 + (R(2,3)+R(3,2))^2);
e(4) = 0.25*sqrt((R(2,1)-R(1,2))^2 + (R(3,1)+R(1,3))^2 + (R(2,3)+R(3,2))^2 + (1-R(1,1)-R(2,2)+R(3,3))^2);

[~, index] = max(e);

switch index
    case 1
        e(1) =                     e(1);
        e(2) = sign(R(3,2)-R(2,3))*e(2);
        e(3) = sign(R(1,3)-R(3,1))*e(3);
        e(4) = sign(R(2,1)-R(1,2))*e(4);
    case 2
        e(1) = sign(R(3,2)-R(2,3))*e(1);
        e(2) =                     e(2);
        e(3) = sign(R(2,1)+R(1,2))*e(3);
        e(4) = sign(R(1,3)+R(3,1))*e(4);
    case 3
        e(1) = sign(R(1,3)-R(3,1))*e(1);
        e(2) = sign(R(2,1)+R(1,2))*e(2);
        e(3) =                     e(3);
        e(4) = sign(R(3,2)+R(2,3))*e(4);
    case 4
        e(1) = sign(R(2,1)-R(1,2))*e(1);
        e(2) = sign(R(1,3)+R(3,1))*e(2);
        e(3) = sign(R(3,2)+R(2,3))*e(3);
        e(4) =                     e(4);
end

e = e/norm(e);
end

