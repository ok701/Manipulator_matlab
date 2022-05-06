% Potential Energy for jinu_manipulator
% Geometric Jacobian and Analytic Jacobian for jinu_manipulator

clc
clear

%------Link Parameters(m)------
l1 = 0.101; 
l2 = 0.2495; 
l3 = 0.25; 
l4 = 0; 
l5 = 0.0956; 
l6 = 0.0655; 

mass_baseLink = 0.52654; % kg
mass_shoulderLink = 0.52932;
mass_armLink = 0.67459;
mass_elbowLink = 0.43335;
mass_forearmLink =  0.13384;
mass_wristLink =  0.11491;
mass_endeff =  0.17466;
% total = 2.58721
 
g= 9.81;

% test
% theta1 = pi/3; theta2 = pi/12; theta3 = pi/11; theta4 = pi/3; theta5 = pi/6; theta6 = pi/7; % joint position ex.
% q1 = 0; q2 = 0; q3 = 0; q4 = 0; q5 = 0; q6 = 0; % joint velocity ex.


%------Foward Kinematics------
syms a alpha d theta theta1 theta2 theta3 theta4 theta5 theta6
A =  [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta) 
      sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)
      0 sin(alpha) cos(alpha) d
      0 0 0 1];

% Denavit-Hartenberg representation old version
% A1 = subs(A, [a,alpha,d,theta], [0,pi/2,l1,theta1]);   
% A2 = subs(A, [a,alpha,d,theta], [l2,0,0,theta2]); 
% A3 = subs(A, [a,alpha,d,theta], [l3,0,0,theta3]);
% A4 = subs(A, [a,alpha,d,theta], [0,pi/2,0,pi/2+theta4]);
% A5 = subs(A, [a,alpha,d,theta], [0,pi/2,l5,pi/2+theta5]);
% A6 = subs(A, [a,alpha,d,theta], [l6,0,0,pi/2+theta6]);

% Denavit-Hartenberg representation new
A1 = subs(A, [a,alpha,d,theta], [0,-pi/2,l1,theta1]);  
A2 = subs(A, [a,alpha,d,theta], [l2,0,0,theta2]);
A3 = subs(A, [a,alpha,d,theta], [l3,0,0,theta3]);
A4 = subs(A, [a,alpha,d,theta], [0,-pi/2,0,-pi/2+theta4]);
A5 = subs(A, [a,alpha,d,theta], [0,pi/2,l5,pi/2+theta5]);
A6 = subs(A, [a,alpha,d,theta], [l6,0,0,pi/2+theta6]);

T_01 = A1;
T_02 = T_01*A2;
T_03 = T_02*A3;
T_04 = T_03*A4;
T_05 = T_04*A5;
T_06 = T_05*A6;

joint1_z = 0;
joint2_z = T_01(3,4);
joint3_z = T_02(3,4);
joint4_z = T_03(3,4);
joint5_z = T_04(3,4);
joint6_z = T_05(3,4);
ee_z = T_06(3,4);

com1 = 0.5*(joint1_z+joint2_z);
com2 = 0.5*(joint2_z+joint3_z);
com3 = 0.5*(joint3_z+joint4_z);
com4 = 0.5*(joint4_z+joint5_z);
com5 = 0.5*(joint5_z+joint6_z);
com6 = 0.5*(joint6_z+ee_z);

%------Potential Energy------
P = mass_shoulderLink*g*com1 + mass_armLink*g*com2 + mass_elbowLink*g*com3 + mass_forearmLink*g*com4 + mass_wristLink*g*com5 + mass_endeff*g*com6   

%%
% End effector position w.r.t base frame
Px = T_06(1,4);  % Px % PR(1)   
Py = T_06(2,4);  % Py % PR(2)
Pz = T_06(3,4);  % Pz % PR(3)

% Rotation matrix w.r.t base frame (Euler order : z->y->x)
r11 = T_06(1,1); r12 = T_06(1,2); r13 = T_06(1,3); 
r21 = T_06(2,1); r22 = T_06(2,2); r23 = T_06(2,3); 
r31 = T_06(3,1); r32 = T_06(3,2); r33 = T_06(3,3);

euler_fi = atan2(r21,r11);  % fi : Roll about z axis % euler_fi
euler_theta = atan2(-r31, cos(euler_fi)*r11 + sin(euler_fi)*r21);  % theta : Pitch about y axis % PR(5)
euler_csi = atan2(sin(euler_fi)*r13 - cos(euler_fi)*r23, -sin(euler_fi)*r12 + cos(euler_fi)*r22);  % csi : Yaw about x axis % PR(4)


%------Jacobian------
z0 = [0 0 1]'; z1 = T_01(1:3,3); z2 = T_02(1:3,3); z3 = T_03(1:3,3); z4 = T_04(1:3,3); z5 = T_05(1:3,3); z6 = T_06(1:3,3); 
o0 = [0 0 0]'; o1 = T_01(1:3,4); o2 = T_02(1:3,4); o3 = T_03(1:3,4); o4 = T_04(1:3,4); o5 = T_05(1:3,4); o6 = T_06(1:3,4); 
Jg = [cross(z0,(o6-o0)) cross(z1,(o6-o1)) cross(z2,(o6-o2)) cross(z3,(o6-o3)) cross(z4,(o6-o4)) cross(z5,(o6-o5))  % Geometric Jacobian
      z0 z1 z2 z3 z4 z5];

B = [cos(euler_fi)*cos(euler_theta) -sin(euler_fi) 0; sin(euler_fi)*cos(euler_theta) cos(euler_fi) 0; -sin(euler_theta) 0 1];  
Ja = [eye(3) zeros(3); zeros(3) inv(B)]*Jg;  % Analytic Jacobian

% Jg*[q1 q2 q3 q4 q5 q6]';
% Ja*[q1 q2 q3 q4 q5 q6]';

