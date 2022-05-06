% caculating Potential Energy for planer_manipulator_CoM.m

clc
clear

g = 9.81;  L1 = 0.25; L2 = 0.25;
m1 = 0.25; m2 = 0.25;
d1 = L1/2; d2 = L2/2; h = 0.06; %h는 폭
del_t = 0.002; 


% Foward Kinematics
A0 = [1 0 0 0; 
      0 1 0 0; 
      0 0 1 0; 
      0 0 0 1];
A1 = [cos(theta1) -sin(theta1) 0 L1*cos(theta1)
      sin(theta1) cos(theta1) 0 L1*sin(theta1)
      0         0           1           0
      0         0           0           1];
A2 = [cos(theta2) -sin(theta2) 0 L2*cos(theta2)
      sin(theta2) cos(theta2) 0 L2*sin(theta2)
      0         0           1           0
      0         0           0           1];

T_00 = A0;
T_01 = A1;
T_02 = T_01*A2;

joint1_z = 0;
joint2_z = T_01(2,4);
joint3_z = T_02(2,4);

com1 = 0.5*(joint1_z+joint2_z);
com2 = 0.5*(joint2_z+joint3_z);


P = m1*g*com1 + m2*g*com2
diff(P,theta1)
diff(P,theta2)

