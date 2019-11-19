t_1 = trsMat(0,30,30);
t_2 = trsMat(0,10,20);
o1 = [0;0;0;1];
o2 = t_1*o1;
o3 = t_2*o2;

Rx = rXMat(90);
r = Rx;

axislen = 10;
standard_basis = [1 0 0 0;0 1 0 0;0 0 1 0;1 1 1 0];    % X Y Z        
standard_basis(1:3,1:3) = eye(3,3)*axislen;

basis1 = Rx*standard_basis;    
basis2 = t_2*t_1*rXMat(90)*rYMat(90)*rZMat(90)*standard_basis;

draw_XYZ(o1,standard_basis,axislen);                                   
draw_XYZ(o2,t_1*basis1,axislen);
draw_XYZ(o3,basis2,axislen);
%draw_vec(o1,vector1,'m');        %vector [1;1;1] in basis1 expressed in standard basis is R1*sample==> sample is the [1;1;1] in basis1

axis('equal');

alpha 0.5;

function draw_XYZ(a,b,axislen)       % a is coordinate origin % b is the basis vectors
ConeLen = axislen./10;
for i=1:3
    xaxisDir = (b(1,i) - a(1))./10;
    yaxisDir = (b(2,i) - a(2))./10;
    zaxisDir = (b(3,i) - a(3))./10;
    if(i == 1)
        plot3([a(1) b(1,i)],[a(2) b(2,i)],[a(3) b(3,i)],'r','LineWidth',3);
        text((a(1)+b(1,i))/2,(a(2)+b(2,i))/2,(a(3)+b(3,i))/2,'X1');
        hold on;
        Cone([b(1,i) b(2,i) b(3,i)],[b(1,i)+xaxisDir b(2,i)+yaxisDir b(3,i)+zaxisDir],[ConeLen 0],30,'r',0,1);
     
    elseif(i == 2)
        plot3([a(1) b(1,i)],[a(2) b(2,i)],[a(3) b(3,i)],'g','LineWidth',3);
        text((a(1)+b(1,i))/2,(a(2)+b(2,i))/2,(a(3)+b(3,i))/2,'Y1');
        hold on;
        Cone([b(1,i) b(2,i) b(3,i)],[b(1,i)+xaxisDir b(2,i)+yaxisDir b(3,i)+zaxisDir],[ConeLen 0],30,'g',0,1);

    else
        plot3([a(1) b(1,i)],[a(2) b(2,i)],[a(3) b(3,i)],'b','LineWidth',3);
        text((a(1)+b(1,i))/2,(a(2)+b(2,i))/2,(a(3)+b(3,i))/2,'Z1');
        hold on;
        Cone([b(1,i) b(2,i) b(3,i)],[b(1,i)+xaxisDir b(2,i)+yaxisDir b(3,i)+zaxisDir],[ConeLen 0],30,'b',0,1);
        hold on;
        Cone([a(1) a(2) a(3)],[b(1,i) b(2,i) b(3,i) ],[5 5],30,'y',1,1);
    end
    hold on
end
end

function draw_vec(a,b,color)       % a is coordinate origin % b is the basis vectors
    plot3([a(1),b(1)],[a(2),b(2)],[a(3),b(3)],color,'LineWidth',3);
    hold on
end

% rotation matrix function defined 
function rx = rXMat(a)
    a = deg2rad(a);
    rx = [1 0 0 0;0 cos(a) -sin(a) 0; 0 sin(a) cos(a) 0;0 0 0 1];
end

function ry = rYMat(a)
    a = deg2rad(a);
    ry = [cos(a) 0 sin(a) 0;0 1 0 0;-sin(a) 0 cos(a) 0;0 0 0 1];
end

function rz = rZMat(a)
    a = deg2rad(a);
    rz = [cos(a) -sin(a) 0 0;sin(a) cos(a) 0 0;0 0 1 0;0 0 0 1];
end

% translation matrix function defined 
function ts = trsMat(x,y,z)
    a = eye(4,4);
    a(1:3,4) = [x y z];
    ts = a;
end
