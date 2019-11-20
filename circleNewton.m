th = -pi:0.1:pi;
r = 10;
x = 3;
y = 3;
len = size(th);
rNumber = (rand(1,len(2))-1/2).*4;
x1 = r*cos(th)+rNumber;
y1 = r*sin(th)+rNumber;




syms x y a b r
func = sqrt((x-a).^2 + (y-b).^2)-r;         % circle equation 
jacMat = jacobian(func,[a,b,r]);            % jacobian matrix 
jacMat = matlabFunction(jacMat);            % 
func = matlabFunction(func);

jac = sym('J',[len(2) 3]);
cur = sym('c',[len(2) 1]);

for i = 1:len(2)
    jac(i,:) = jacMat(a,b,x1(i),y1(i));
    cur(i)= func(a,b,r,x1(i),y1(i));
end
jacf = matlabFunction(jac);
currentval = matlabFunction(cur);

tp1 = jacf(-1,-1);
tp2 = currentval(-1,-1,8);

result1 = [-1;-1;8]-inv(tp1'*tp1)*tp1'*tp2;

tp3 = jacf(result1(1),result1(2));
tp4 = currentval(result1(1),result1(2),result1(3));
result2 = result1-inv(tp3'*tp3)*tp3'*tp4;

tp5 = jacf(result2(1),result2(2));
tp6 = currentval(result2(1),result2(2),result2(3));
result3 = result2-inv(tp5'*tp5)*tp5'*tp6;





figure(2);

plot(x1,y1,'*r');
hold on 
circle(result1(1),result1(2),result1(3),'b');
circle(-1,-1,8,'b');
circle(result2(1),result2(2),result2(3),'g');
circle(result3(1),result3(2),result3(3),'m');


function h = circle(x,y,r,color)
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit,color);
hold off
end
