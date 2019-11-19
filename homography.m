i1 = imread('/home/oggyu/Pictures/perspective_test.jpg');
i2 = rgb2gray(i1);
figure(3);
image(i1);
x = [89.2097;
  388.7488;
  510.8687;
  213.6336];

y = [296.9927;
  165.4300;
  316.0598;
  504.8236];

distorted_XY = zeros(8,1);
for i= 1:4
    distorted_XY(2*i-1,1)= x(i);
    distorted_XY(2*i,1) = y(i);
end
undistorted_XY = zeros(8,1);
% undistorted_XY(1) = x(1);
% undistorted_XY(2) = y(1);
% undistorted_XY(3) = x(2);
% undistorted_XY(4) = y(1);
% undistorted_XY(5) = x(2);
% undistorted_XY(6) = y(3);
% undistorted_XY(7) = x(1);
% undistorted_XY(8) = y(3);

undistorted_XY(1:8,1) = [0 0 200 0 200 100 0 100];

pin = [x(1) y(1);x(2) y(2);x(3) y(3);x(4) y(4)];

k = KnownConstants(distorted_XY,undistorted_XY);

[U,S,V] = svd(k);
V1 = V ;
V = V(:,9);
V = V/V(9);
projectMat = [V(1:3)';V(4:6)';V(7:9)'];
projectMat = projectMat';
tform = projective2d(projectMat);

outputImage = imwarp(i2,tform);
figure(2);
imshow(outputImage);

outputimage2 = zeros(654,1000);
for i = 1:654
    for j = 1 : 1000
        temp = projectMat*[i;j;1];
        temp = round(temp);
        if(temp(1)<700 && temp(1) > 0 && temp(2)>0 && temp(2)<1200)
            outputimage2(temp(1),temp(2)) = i1(i,j);
        end
    end
end

figure(4);
imshow(outputimage2);


function retMat = KnownConstants(d,ud)
    a = zeros(8,9);
    for i = 1:4
        ind = 2*i-1;
        a(ind,:)=[-d(ind) -d(ind+1) -1 0 0 0 ud(ind)*d(ind) ud(ind)*d(ind+1) ud(ind)];
        a(ind+1,:) = [0 0 0 -d(ind) -d(ind+1) -1 d(ind)*ud(ind+1) d(ind+1)*ud(ind+1) ud(ind+1)];
    end
    retMat = a;
end
