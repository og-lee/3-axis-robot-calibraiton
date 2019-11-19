[x,y] = meshgrid(-5:0.5:5,-5:0.5:5);
z = sin(x).*cos(y);
surf(x,y,z);
hold on ;

syms x y
f = sin(x).*cos(y);
a = hessian(f);
f1 = matlabFunction(a);
f2 = matlabFunction(f);
xdiff = matlabFunction(diff(f,x));
ydiff = matlabFunction(diff(f,y));
xval = pi./3;
yval = pi./6;

k = f1(xval,yval);
syms x1 y1
z1 = f2(xval,yval)+xdiff(xval,yval).*(x1-xval)+ydiff(xval,yval).*(y1-yval)+k(1,1)./2.*(x1-xval).^2+k(1,2).*(x1-xval).*(y1-yval)+k(2,2)./2.*(y1-yval).^2;
f3 = matlabFunction(z1);

[x,y] = meshgrid(-3:0.5:3,-3:0.5:3);
surf(x,y,f3(x,y));
hold on ;
plot3(xval,yval,f2(xval,yval),'r*');

