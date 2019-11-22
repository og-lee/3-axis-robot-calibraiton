
% get 3point circleCenter (3x2 matrix X and Y)
% calculate the equation AX = B using inverse matrix 

function [A,B,R] = dot3Circle(circleCenter)
    a = [-2*circleCenter(1,1) -2*circleCenter(1,2) 1 ; ...
     -2*circleCenter(2,1) -2*circleCenter(2,2) 1 ; ...
     -2*circleCenter(3,1) -2*circleCenter(3,2) 1 ; ];
 
    b = [-(circleCenter(1,1)^2 + circleCenter(1,2)^2);...
     -(circleCenter(2,1)^2 + circleCenter(2,2)^2);...
     -(circleCenter(3,1)^2 + circleCenter(3,2)^2)];

    coeff = inv(a)*b;
    A = coeff(1);
    B = coeff(2);
    R = sqrt(coeff(1)^2 + coeff(2)^2 - coeff(3));
end