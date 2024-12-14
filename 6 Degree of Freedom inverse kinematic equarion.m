clc;close;clear;

syms q1 q2 q3 q4 q5 q6
DH_table=[q1      280  0    -pi/2;
         q2-pi/2  0    210   0;
         q3-pi/2  0    75   -pi/2;
         q4+pi    210  0    -pi/2;
         q5       0    0     pi/2;
         q6       70   0     0];

A1=[cos(DH_table(1,1)) -sin(DH_table(1,1))*cos(DH_table(1,4)) sin(DH_table(1,1))*sin(DH_table(1,4)) DH_table(1,3)*cos(DH_table(1,1));
    sin(DH_table(1,1)) cos(DH_table(1,1))*cos(DH_table(1,4)) -cos(DH_table(1,1))*sin(DH_table(1,4)) DH_table(1,3)*sin(DH_table(1,1));
    0 sin(DH_table(1,4)) cos(DH_table(1,4)) DH_table(1,2);
    0 0 0 1];

A2=[cos(DH_table(2,1)) -sin(DH_table(2,1))*cos(DH_table(2,4)) sin(DH_table(2,1))*sin(DH_table(2,4)) DH_table(2,3)*cos(DH_table(2,1));
    sin(DH_table(2,1)) cos(DH_table(2,1))*cos(DH_table(2,4)) -cos(DH_table(2,1))*sin(DH_table(2,4)) DH_table(2,3)*sin(DH_table(2,1));
    0 sin(DH_table(2,4)) cos(DH_table(2,4)) DH_table(2,2);
    0 0 0 1];

A3=[cos(DH_table(3,1)) -sin(DH_table(3,1))*cos(DH_table(3,4)) sin(DH_table(3,1))*sin(DH_table(3,4)) DH_table(3,3)*cos(DH_table(3,1));
    sin(DH_table(3,1)) cos(DH_table(3,1))*cos(DH_table(3,4)) -cos(DH_table(3,1))*sin(DH_table(3,4)) DH_table(3,3)*sin(DH_table(3,1));
    0 sin(DH_table(3,4)) cos(DH_table(3,4)) DH_table(3,2);
    0 0 0 1];

A4=[cos(DH_table(4,1)) -sin(DH_table(4,1))*cos(DH_table(4,4)) sin(DH_table(4,1))*sin(DH_table(4,4)) DH_table(4,3)*cos(DH_table(4,1));
    sin(DH_table(4,1)) cos(DH_table(4,1))*cos(DH_table(4,4)) -cos(DH_table(4,1))*sin(DH_table(4,4)) DH_table(4,3)*sin(DH_table(4,1));
    0 sin(DH_table(4,4)) cos(DH_table(4,4)) DH_table(4,2);
    0 0 0 1];

A5=[cos(DH_table(5,1)) -sin(DH_table(5,1))*cos(DH_table(5,4)) sin(DH_table(5,1))*sin(DH_table(5,4)) DH_table(5,3)*cos(DH_table(5,1));
    sin(DH_table(5,1)) cos(DH_table(5,1))*cos(DH_table(5,4)) -cos(DH_table(5,1))*sin(DH_table(5,4)) DH_table(5,3)*sin(DH_table(5,1));
    0 sin(DH_table(5,4)) cos(DH_table(5,4)) DH_table(5,2);
    0 0 0 1];

A6=[cos(DH_table(6,1)) -sin(DH_table(6,1))*cos(DH_table(6,4)) sin(DH_table(6,1))*sin(DH_table(6,4)) DH_table(6,3)*cos(DH_table(6,1));
    sin(DH_table(6,1)) cos(DH_table(6,1))*cos(DH_table(6,4)) -cos(DH_table(6,1))*sin(DH_table(6,4)) DH_table(6,3)*sin(DH_table(6,1));
    0 sin(DH_table(6,4)) cos(DH_table(6,4)) DH_table(6,2);
    0 0 0 1];

T60=A1*A2*A3*A4*A5*A6;
T63=A4*A5*A6;
T40=A1*A2*A3*A4;
T30=A1*A2*A3;

disp('wrist position');
wp=simplify(T40(1:3,4));
disp(wp)

O60=[-163.27;-189.71;709.37];
R60=[0.7772 0.2195 -0.5897;-0.5708 0.64604 -0.5139;0.2649 0.7360 0.6230];

% inverse position problem: calculating the first 3 joint variables using the wrist position
Oc0=O60-70*R60(:,3);
xc=Oc0(1);
yc=Oc0(2);
zc=Oc0(3);

theta1=atan2(yc,xc)+pi

A=2*(210^2);
B=2*75*210;
C=xc^2+yc^2+(zc-280)^2-210^2-210^2-75^2;
D=roots([A^2+B^2,-2*A*C,C^2-B^2]);
theta3=atan2(sqrt(1-D(2)^2),D(2))

AA=210+210*cos(theta3)+75*sin(theta3);
BB=-210*sin(theta3)+75*cos(theta3);
CC=zc-280;
DD=roots([AA^2+BB^2,-2*AA*CC,CC^2-BB^2]);
theta2=atan2(-sqrt(1-DD(2)^2),DD(2))

% inverse orientation problem: calculating the last 3 joint variables using R63
q1=theta1;
q2=theta2;
q3=theta3;
temp=eval(T30);
R30=temp(1:3,1:3);

R63=R30*R60;
disp('R63 symbolic');
disp(simplify(T63(1:3,1:3)));

theta5=atan2(sqrt(R63(3,1)^2+R63(3,2)^2),R63(3,3))
theta6=atan2(R63(3,2),-R63(3,1))
theta4=atan2(-R63(2,3),-R63(1,3))

q4=theta4;
q5=theta5;
q6=theta6;
