
clear 
clc

syms a alph d theta
f = [cosd(theta) -sind(theta).*cosd(alph) sind(theta).*sind(alph) a.*cosd(theta);
     sind(theta) cosd(theta).*cosd(alph) -cosd(theta).*sind(alph) a.*sind(theta);
      0 sind(alph) cosd(alph) d;
      0 0 0 1];
func1 = matlabFunction(f);


d1 = 50;          %unkown
d2 = -50;          %unkown
%------------------------------d's 
a1 = 0;
a2 = 50;          %unknown
a3 = 0;
%------------------------------a's
al1 = DtoR(90);
al2 = DtoR(90);
al3 = DtoR(0);
%------------------------------alpha's
L(1) = Link([0,d1,a1,al1,0],'standard');   % rotation link 
L(2) = Link([0,d2,a2,al2,0],'standard');   % rotation link
L(3) = Link([0,0,a3,al3,1],'standard');    % prismatic link
L(3).qlim=[0,6000];

% D L1 L3 L4 tx ty tz alp bet gam
l1 = [a1,90,d1];
l2 = [a2,90,d2];

simulationLen= 4000;
Rot2v = eye(4);
Ry = RotY3(DtoR(-90));
Rx = RotX3(DtoR(0));
Rot2v(1:3,1:3)= Ry*Rx;
Rot2v(:,4) = [simulationLen;0;0;1];

norVec1 = ones(4,1);
norVec1(1:3,:) = Ry*Rx*[0;0;1]; 
% norVec2 = ones(4,1);
% norVec2(1:3,:) = Ry*Rx*[0;1;0];
% 
% 
% Rx2 = RotX3(DtoR(45));
% vec2 = Ry*Rx2*[0;0;1];

My_Robot = SerialLink(L,'name','Three Link');
My_Robot.teach;
hold on;
%drawWith_Nor_Point([-1;0;0],[1000,0,0],4000,'y');
drawWith_Nor_Point(norVec1(1:3),[simulationLen,0,0],6000,'y');
hold on;
% drawWith_Nor_Point(vec2(1:3),[simulationLen,0,0],6000,'y');
% hold on 
% variable parameter theta1 2 and prismatic length 


grid_dim = 5;
samples = zeros(grid_dim*grid_dim,3);
index = 1;

for i=1:grid_dim
    for j = 1:grid_dim
        samples(index,1) = 60*(j-grid_dim/2)/grid_dim*2;
        samples(index,2) = 60*(i-grid_dim/2)/grid_dim*2;
        index = index + 1; 
    end
end
 
 % NO NEED to use ROBOT TOOLBOX ANYMORE
 % [simulationPoints,samples(:,3)] = makeSampleData(L(1),L(2),samples,grid_dim,normalVec1,normalVec2);
 [simul,newSamples] = makeSampleData(l1,l2,samples,grid_dim,norVec1,simulationLen);


R= eye(4);
R(1:3,1:3) = RotY3(DtoR(-90))*RotX3(DtoR(0)); 
T = eye(4);
T(:,4) = [simulationLen 0 0 1];
H = T*R;

plane_Points = zeros(grid_dim*grid_dim,4);
for i = 1:grid_dim*grid_dim
    plane_Points(i,:)= inv(H)*simul(i,:)';
end


% adding error process 
plane_PointsWithError = zeros(grid_dim*grid_dim,4);
error = zeros(grid_dim*grid_dim,3);
for i = 1:grid_dim*grid_dim
    mini = -2;
    maxi = 2;
    random = mini + (maxi-mini)*rand(1);
    random1 = mini + (maxi-mini)*rand(1);         % random -1 or 1 
% random -1 or 1 
    random2 = mini + (maxi-mini)*rand(1); 
    plane_PointsWithError(i,1) = plane_Points(i,1)+random;
    plane_PointsWithError(i,2) = plane_Points(i,2)+random1;
    plane_PointsWithError(i,3) = plane_Points(i,3)+random2;
    error(i,:) = [random;random1;random2];
end

%   figure(2)
%   plot(plane_Points(:,1),plane_Points(:,2),'bo');
%   hold on;
%   plot(plane_PointsWithError(:,1),plane_PointsWithError(:,2),'ro');

% d(1) = D 
% d(2) = L1 
% d(3) = L3 
% d(4) = L4 


% d(5) = tx 
% d(6) = ty d
% d(7) = tz 
% d(8) = alp 
% d(9) = bet 
% d(10) = gam    


% known = Xn Yn t1 t2 L5 
t1 = newSamples(:,1)';                
t2 = newSamples(:,2)';               
L5 = newSamples(:,3)';

%  Xn = plane_Points(:,1)';
%  Yn = plane_Points(:,2)';

% Zn = plane_Points(:,3)';
  Zn = plane_PointsWithError(:,3)';
  Xn = plane_PointsWithError(:,1)';                
  Yn = plane_PointsWithError(:,2)';       
func = @(d)(d(2) + (4967757600021511.*d(3))/81129638414606681695789005144064 - d(7) + Xn.*sin((pi.*d(9))/180) + d(4).*sin((pi.*(t2 + 90))/180) - (cos((pi.*(t2 + 90))/180) - 24678615572571482867467662723121/6582018229284824168619876730229402019930943462534319453394436096).*(d(1) + L5) - Zn.*cos((pi.*d(9))/180).*cos((pi.*d(10))/180) - Yn.*cos((pi.*d(9))/180).*sin((pi.*d(10))/180)).*(conj(d(2)) + (4967757600021511.*conj(d(3)))/81129638414606681695789005144064 - conj(d(7)) + conj(Xn).*sin((pi.*conj(d(9)))/180) - (conj(d(1)) + conj(L5)).*(cos((pi.*(conj(t2) + 90))/180) - 24678615572571482867467662723121/6582018229284824168619876730229402019930943462534319453394436096) + sin((pi.*(conj(t2) + 90))/180).*conj(d(4)) - conj(Zn).*cos((pi.*conj(d(9)))/180).*cos((pi.*conj(d(10)))/180) - conj(Yn).*cos((pi.*conj(d(9)))/180).*sin((pi.*conj(d(10)))/180)) + (d(6) + d(3).*cos((pi.*t1)/180) + Yn.*(cos((pi.*d(8))/180).*cos((pi.*d(10))/180) + sin((pi.*d(8))/180).*sin((pi.*d(9))/180).*sin((pi.*d(10))/180)) - Zn.*(cos((pi.*d(8))/180).*sin((pi.*d(10))/180) - cos((pi.*d(10))/180).*sin((pi.*d(8))/180).*sin((pi.*d(9))/180)) + (d(1) + L5).*((4967757600021511.*cos((pi.*t1)/180))/81129638414606681695789005144064 + (4967757600021511.*cos((pi.*t1)/180).*cos((pi.*(t2 + 90))/180))/81129638414606681695789005144064 - sin((pi.*t1)/180).*sin((pi.*(t2 + 90))/180)) - (4967757600021511.*d(4).*cos((pi.*t1)/180).*sin((pi.*(t2 + 90))/180))/81129638414606681695789005144064 - d(4).*sin((pi.*t1)/180).*cos((pi.*(t2 + 90))/180) + Xn.*cos((pi.*d(9))/180).*sin((pi.*d(8))/180)).*(conj(d(6)) + (conj(d(1)) + conj(L5)).*((4967757600021511.*cos((pi.*conj(t1))/180))/81129638414606681695789005144064 + (4967757600021511.*cos((pi.*(conj(t2) + 90))/180).*cos((pi.*conj(t1))/180))/81129638414606681695789005144064 - sin((pi.*(conj(t2) + 90))/180).*sin((pi.*conj(t1))/180)) + conj(Yn).*(cos((pi.*conj(d(8)))/180).*cos((pi.*conj(d(10)))/180) + sin((pi.*conj(d(8)))/180).*sin((pi.*conj(d(9)))/180).*sin((pi.*conj(d(10)))/180)) - conj(Zn).*(cos((pi.*conj(d(8)))/180).*sin((pi.*conj(d(10)))/180) - cos((pi.*conj(d(10)))/180).*sin((pi.*conj(d(8)))/180).*sin((pi.*conj(d(9)))/180)) + conj(d(3)).*cos((pi.*conj(t1))/180) - cos((pi.*(conj(t2) + 90))/180).*conj(d(4)).*sin((pi.*conj(t1))/180) - (4967757600021511.*sin((pi.*(conj(t2) + 90))/180).*conj(d(4)).*cos((pi.*conj(t1))/180))/81129638414606681695789005144064 + conj(Xn).*cos((pi.*conj(d(9)))/180).*sin((pi.*conj(d(8)))/180)) + (d(5) - d(3).*sin((pi.*t1)/180) - Yn.*(cos((pi.*d(10))/180).*sin((pi.*d(8))/180) - cos((pi.*d(8))/180).*sin((pi.*d(9))/180).*sin((pi.*d(10))/180)) + Zn.*(sin((pi.*d(8))/180).*sin((pi.*d(10))/180) + cos((pi.*d(8))/180).*cos((pi.*d(10))/180).*sin((pi.*d(9))/180)) - (d(1) + L5).*((4967757600021511.*sin((pi.*t1)/180))/81129638414606681695789005144064 + cos((pi.*t1)/180).*sin((pi.*(t2 + 90))/180) + (4967757600021511.*sin((pi.*t1)/180).*cos((pi.*(t2 + 90))/180))/81129638414606681695789005144064) - d(4).*cos((pi.*t1)/180).*cos((pi.*(t2 + 90))/180) + (4967757600021511.*d(4).*sin((pi.*t1)/180).*sin((pi.*(t2 + 90))/180))/81129638414606681695789005144064 + Xn.*cos((pi.*d(8))/180).*cos((pi.*d(9))/180)).*(conj(d(5)) - conj(d(3)).*sin((pi.*conj(t1))/180) - (conj(d(1)) + conj(L5)).*((4967757600021511.*sin((pi.*conj(t1))/180))/81129638414606681695789005144064 + (4967757600021511.*cos((pi.*(conj(t2) + 90))/180).*sin((pi.*conj(t1))/180))/81129638414606681695789005144064 + sin((pi.*(conj(t2) + 90))/180).*cos((pi.*conj(t1))/180)) - conj(Yn).*(cos((pi.*conj(d(10)))/180).*sin((pi.*conj(d(8)))/180) - cos((pi.*conj(d(8)))/180).*sin((pi.*conj(d(9)))/180).*sin((pi.*conj(d(10)))/180)) + conj(Zn).*(sin((pi.*conj(d(8)))/180).*sin((pi.*conj(d(10)))/180) + cos((pi.*conj(d(8)))/180).*cos((pi.*conj(d(10)))/180).*sin((pi.*conj(d(9)))/180)) - cos((pi.*(conj(t2) + 90))/180).*conj(d(4)).*cos((pi.*conj(t1))/180) + (4967757600021511.*sin((pi.*(conj(t2) + 90))/180).*conj(d(4)).*sin((pi.*conj(t1))/180))/81129638414606681695789005144064 + conj(Xn).*cos((pi.*conj(d(8)))/180).*cos((pi.*conj(d(9)))/180))...
     +((d(2)-53)^2)*100+((-d(3)-53)^2)*100;


% exact value 
%d = [10,50,-50,50,...
%    4000,0,0,0,-80,0];

% with estimation 
d = [5,34,-40,55,...
    3000,50,50,10,-90,10];

% d = [0,0,0,0,0,0,0,0,0,0];
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxFunctionEvaluations',simulationLen,...
    'MaxIterations',5000,'OptimalityTolerance',1e-20,...
    'FunctionTolerance',1e-20);
% 'levenberg-marquardt'
% l1 = [a1,90,d1];
% l2 = [a2,90,d2];
 k = lsqnonlin(func,d,[],[],options);

 projectedPoints = ones(grid_dim*grid_dim,4);
 projectedErrors = ones(grid_dim*grid_dim,1);
 for i =1: grid_dim*grid_dim
     dh1 = func1(0,90,k(2)-k(7),newSamples(i,1));
     dh2 = func1(k(4),90,k(3)+k(6),newSamples(i,2)+90);
     projectedPoints(i,:) = dh1*dh2*[0;0;k(1)+newSamples(i,3);1];
     projectedErrors(i) = sqrt(sum((projectedPoints(i,:)-simul(i,:)).^2));
 end
 
ori = [10;50;-50;50;simulationLen;0;0;0;-90;0];
fprintf(' estimation          optimized          original           error  \n');
fprintf(' D : %f ==>     %f      : %f          %f \n',d(1),k(1),ori(1),abs(ori(1)-k(1)));          
fprintf(' L1: %f ==>     %f      : %f          %f \n',d(2),k(2),ori(2),abs(ori(2)-k(2)));
fprintf(' L3: %f ==>     %f      : %f          %f \n',d(3),k(3),ori(3),abs(ori(3)-k(3)));
fprintf(' L4: %f ==>     %f      : %f          %f \n',d(4),k(4),ori(4),abs(ori(4)-k(4)));
fprintf(' tx: %f ==>     %f      : %f          %f \n',d(5),k(5),ori(5),abs(ori(5)-k(5)));
fprintf(' ty: %f ==>     %f      : %f          %f \n',d(6),k(6),ori(6),abs(ori(6)-k(6)));
fprintf(' tz: %f ==>     %f      : %f          %f \n',d(7),k(7),ori(7),abs(ori(7)-k(7)));
fprintf(' rz: %f ==>    %f      : %f          %f \n',d(8),k(8),ori(8),abs(ori(8)-k(8)));
fprintf(' ry: %f ==>    %f      : %f          %f \n',d(9),k(9),ori(9),abs(ori(9)-k(9)));
fprintf(' rx: %f ==>    %f      : %f          %f \n',d(10),k(10),ori(10),abs(ori(10)-k(10)));
fprintf(' L1 - tz: %f ==>    %f      : %f          %f \n',d(10),k(2)-k(7), ori(2)-ori(7),abs((ori(2)-ori(7))-(k(2)-k(7))));

errData = zeros(8,1);
% for i = 1:10
%     if (i ~= 2 && i ~= 7 && i ~= 8 && i~=10)
%         per = (ori(i)-k(i));
%         fprintf('%f,\n',per);
%         errData(i) = per;
%     elseif(i == 2)
%         fprintf('%f,\n',abs((ori(2)-ori(7))-(k(2)-k(7))));
%         errData(i) = abs((ori(2)-ori(7))-(k(2)-k(7)));
%     elseif(i == 8)
%         fprintf('%f,\n',abs((k(8)+k(10))));
%         errData(i) = abs((k(8)+k(10)));
%     end    
% end

for i=1:grid_dim*grid_dim
    fprintf('%f \n',projectedErrors(i));
end

figure(3)
plot3(projectedPoints(:,1),projectedPoints(:,2),projectedPoints(:,3),'ro');
hold on ;
plot3(simul(:,1),simul(:,2),simul(:,3),'bo');

