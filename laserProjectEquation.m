
syms a alpha d theta
f = [cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha) a*cosd(theta);
     sind(theta) cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha) a*sind(theta);
      0 sind(alpha) cosd(alpha) d;
      0 0 0 1];
vpa(f,3);
func1 = matlabFunction(f);
syms  L1 L3 L4 L5 D t1 t2 
m1 = vpa(func1(0,-90,L1,t1),3);
%m1 = func1(0,-90,100,30);
m2 = vpa(func1(L4,90,L3,t2),3);
m3 = vpa(func1(0,0,L5 + D,0),3);

final = vpa(m1*m2*m3*[0;0;0;1],3);


