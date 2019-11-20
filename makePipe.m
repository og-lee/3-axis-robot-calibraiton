clear
clc

pipImg = imread('/home/oggyu/Development/MatlabDev/PipeProject/image/pipe2.jpg');
pipImg = rgb2gray(pipImg);

% pipImg = edge(pipImg,'canny');

[centers, radii, metric] = imfindcircles(pipImg,[1 15]);

imshow(pipImg);

circleCenter = centers(1:3,:);
circleRadius = radii(1:3);


viscircles(circleCenter,circleRadius,'EdgeColor','r'); 



% todo check it with new picture %