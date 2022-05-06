% necessary joint torque caculation for gravity compensation

clc
clear

syms theta1 theta2 theta3 theta4 theta5 theta6
% Potential Energy from jinu_manipulator.m
P = (1225462127381439021*sin(theta4 - pi/2)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/5629499534213120000 - (28279155227032207*cos(theta2)*sin(theta3))/18014398509481984 - (28279155227032207*cos(theta3)*sin(theta2))/18014398509481984 - (31589558418310719*sin(theta6 + pi/2)*(cos(theta4 - pi/2)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4 - pi/2)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))))/562949953421312000 - (1225462127381439021*cos(theta4 - pi/2)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/5629499534213120000 - (26324107705402184097*sin(theta2))/9007199254740992000 - (31589558418310719*cos(theta5 + pi/2)*cos(theta6 + pi/2)*(cos(theta4 - pi/2)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4 - pi/2)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))))/562949953421312000 + 8014176560159218751/4503599627370496000;

%------torque caculation by partial diff. eq.------
tou1 = diff(P,theta1); 
tou2 = diff(P,theta2);
tou3 = diff(P,theta3);
tou4 = diff(P,theta4);
tou5 = diff(P,theta5);
tou6 = diff(P,theta6);

shrtTou1 = vpa(tou1,5) % ratio to decimals
shrtTou2 = vpa(tou2,5)
shrtTou3 = vpa(tou3,5)
shrtTou4 = vpa(tou4,5)
shrtTou5 = vpa(tou5,5)
shrtTou6 = vpa(tou6,5)

%% Go over with the code
A1 = subs(shrtTou2, [theta1,theta2,theta3,theta4,theta5,theta6], [0,deg2rad(-60),deg2rad(60),deg2rad(30),0,0]); % joint angles you want to observe
A2 = subs(shrtTou3, [theta1,theta2,theta3,theta4,theta5,theta6], [0,deg2rad(-60),deg2rad(60),deg2rad(30),0,0]);
A3 = subs(shrtTou4, [theta1,theta2,theta3,theta4,theta5,theta6], [0,deg2rad(-60),deg2rad(60),deg2rad(30),0,0]);
A4 = subs(shrtTou5, [theta1,theta2,theta3,theta4,theta5,theta6], [0,deg2rad(-60),deg2rad(60),deg2rad(30),0,0]);
A5 = subs(shrtTou6, [theta1,theta2,theta3,theta4,theta5,theta6], [0,deg2rad(-60),deg2rad(60),deg2rad(30),0,0]);

t2 = vpa(A1,5); % ratio to decimals
t3 = vpa(A2,5);
t4 = vpa(A3,5);
t5 = vpa(A4,5);
t6 = vpa(A5,5);