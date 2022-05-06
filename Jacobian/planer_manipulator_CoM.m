% 2-link planer manipulator CoM Jacobian control

clc
clear

g = 9.81;  L1 = 0.25; L2 = 0.25;
m1 = 0.25; m2 = 0.25;
d1 = L1/2; d2 = L2/2;
h = 0.06; % 폭
step_time = 500;
del_t = 0.002;

%------Initial Settings------
Init_th = [pi/3 -pi*2/4]; % theta1 theta2
I1 = (m1*(L1^2+h^2))/12;
I2 = (m2*(L2^2+h^2))/12;
A0 = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
A1 = [cos(Init_th(1)) -sin(Init_th(1)) 0 L1*cos(Init_th(1));
    sin(Init_th(1)) cos(Init_th(1)) 0 L1*sin(Init_th(1));
    0 0 1 0; 0 0 0 1];
A2 = [cos(Init_th(2)) -sin(Init_th(2)) 0 L2*cos(Init_th(2));
    sin(Init_th(2)) cos(Init_th(2)) 0 L2*sin(Init_th(2));
    0 0 1 0; 0 0 0 1];

T00 = A0;
T01 = T00*A1;
T02 = T01*A2;
T12 = T02-T01;

CoM1 = [0.5*T01(1,4) 0.5*T01(2,4) 0];
CoM2 = [(T01(1,4)+T02(1,4))/2 (T01(2,4)+T12(2,4))/2 0];

m = m1+m2;
M1 = m;
M2 = m2;

CoM_total = [(m1*CoM1(1)+m2*CoM2(1))/m (m1*CoM1(2)+m2*CoM2(2))/m];
init_CoM = CoM_total;  % 0.1768    0.0884

CoM_x(1) = init_CoM(1);
CoM_y(1) = init_CoM(2);

Ax = [0 T01(1,4) T02(1,4)];
Ay = [0 T01(2,4) T02(2,4)];

theta_1(1) = Init_th(1);
theta_1_dot(1) = 0;
theta_1_ddot(1) = 0;
theta_1_ddot(2) = 0;
theta_2(1) = Init_th(2);
theta_2_dot(1) = 0;
theta_2_ddot(1) = 0;
theta_2_ddot(2) = 0;

%-----Joint Space Control <Inverse Dyamics>------ % this if to determine input CoM
for i = 1:1:step_time    
    t = del_t*i;
   
    theta_1(i) = Init_th(1)+pi/180*10*0.5*(1-cos(pi*(i-1)/step_time)); % joint 1 sinusoidal 10도
    
    if i >= 2
        theta_1_dot(i) = (theta_1(i) - theta_1(i-1))/del_t;

    end
    if i >= 3
        theta_1_ddot(i) = (theta_1_dot(i) - theta_1_dot(i-1))/del_t;
    end

    theta_2(i) = Init_th(2)+pi/180*30*0.5*(1-cos(pi*(t-1)/step_time)); % joint 2 30도
    if i >= 2
        theta_2_dot(i) = (theta_2(i) - theta_2(i-1))/del_t;

    end
    if i >= 3
        theta_2_ddot(i) = (theta_2_dot(i) - theta_2_dot(i-1))/del_t;
    end

    A0 = [1 0 0 0;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];
    A1 = [cos(theta_1(i)) -sin(theta_1(i)) 0 L1*cos(theta_1(i));
        sin(theta_1(i)) cos(theta_1(i)) 0 L1*sin(theta_1(i));
        0         0           1           0;
        0         0           0           1];
    A2 = [cos(theta_2(i)) -sin(theta_2(i)) 0 L2*cos(theta_2(i));
        sin(theta_2(i)) cos(theta_2(i)) 0 L2*sin(theta_2(i));
        0         0           1           0;
        0         0           0           1];

    T00 = A0;
    T01 = T00*A1;
    T02 = T01*A2;
    T12 = T02-T01;

    CoM1 = [0.5*T01(1,4) 0.5*T01(2,4) 0];
    CoM2 = [T01(1,4)+0.5*T12(1,4) T01(2,4)+0.5*T12(2,4) 0];

    m = m1+m2;

    CoM_x(i) = (m1*CoM1(1)+m2*CoM2(1))/m;
    CoM_y(i) = (m1*CoM1(2)+m2*CoM2(2))/m;
end

%-----CoM control Initial parameters------
f1 = figure()

Kp = [2000 5000];
Kd = [60 150];

act_th_dot(1,1) = 0;
act_th_dot(2,1) = 0;
act_th(1,1) = Init_th(1);
act_th(2,1) = Init_th(2);
act_CoM_x(1) = init_CoM(1);
act_CoM_y(1) = init_CoM(2);
act_CoM_x_dot(1) = 0;
act_CoM_y_dot(1) = 0;

%-----CoM control------
for i = 1:1:step_time

    A0 = [1 0 0 0;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];
    A1 = [cos(act_th(1,i)) -sin(act_th(1,i)) 0 L1*cos(act_th(1,i));
        sin(act_th(1,i)) cos(act_th(1,i)) 0 L1*sin(act_th(1,i));
        0         0           1           0;
        0         0           0           1];
    A2 = [cos(act_th(2,i)) -sin(act_th(2,i)) 0 L2*cos(act_th(2,i));
        sin(act_th(2,i)) cos(act_th(2,i)) 0 L2*sin(act_th(2,i));
        0         0           1           0;
        0         0           0           1];

    T00 = A0;
    T01 = T00*A1;
    T02 = T01*A2;
    T12 = T02-T01;


    G1(i) = m1*g*d1*cos(act_th(1,i))+m2*g*(L1*cos(act_th(1,i)))+m2*g*d2*cos(act_th(1,i)+act_th(2,i)); % Geometrically solved
    G2(i) = m2*g*d2*cos(act_th(1,i)+act_th(2,i));

    G1_partialdiff(i) = (2943*cos(act_th(1,i)))/3200 + (981*cos(act_th(1,i))*cos(act_th(2,i)))/3200 - (981*sin(act_th(1,i))*sin(act_th(2,i)))/3200; % Partial diff. solved
    G2_partialdiff(i) = (981*cos(act_th(1,i))*cos(act_th(2,i)))/3200 - (981*sin(act_th(1,i))*sin(act_th(2,i)))/3200;

    G = [G1_partialdiff(i);G2_partialdiff(i)];

    CoM1 = [0.5*T01(1,4) 0.5*T01(2,4) 0];
    CoM2 = [T01(1,4)+0.5*T12(1,4) T01(2,4)+0.5*T12(2,4) 0];

    m = m1+m2;
    act_CoM_x(i) = (m1*CoM1(1)+m2*CoM2(1))/m;
    act_CoM_y(i) = (m1*CoM1(2)+m2*CoM2(2))/m;

    C1 = [act_CoM_x(i) act_CoM_y(i) 0];
    C2 = [T01(1,4)+0.5*T12(1,4) T01(2,4)+0.5*T12(2,4) 0];


    if i >= 2
        act_CoM_x_dot(i) = (act_CoM_x(i) - act_CoM_x(i-1))/del_t;
        act_CoM_y_dot(i) = (act_CoM_y(i) - act_CoM_y(i-1))/del_t;
    end

    Ax = [0 T01(1,4) T02(1,4)];
    Ay = [0 T01(2,4) T02(2,4)];
    a0 = [T00(1,3); T00(2,3); T00(3,3)];
    C1_P0 = [C1(1)-T00(1,4); C1(2)-T00(2,4); C1(3)-T00(3,4)];
    a1 = [T01(1,3); T01(2,3); T01(3,3)];
    C2_P1 = [C2(1)-T01(1,4); C2(2)-T01(2,4); C2(3)-T01(3,4)];

    J1 = M1/m*(cross(a0,C1_P0));
    J2 = M2/m*(cross(a1,C2_P1));

    J = [J1 J2];

    F = [Kp(1)*(CoM_x(i)-act_CoM_x(i))-Kd(1)*(act_CoM_x_dot(i)); Kp(2)*(CoM_y(i)-act_CoM_y(i))-Kd(2)*(act_CoM_y_dot(i)); 0];
    T_CoM(1:2,i) = transpose(J)*F + G; % CoM Jacobian Torque
    % transJ =  transpose(J); %확인용 % show_F = F; %확인용 % Tau_G = G; %확인용


    % actual joint pos anfer torque input
    M11 = m1*d2^2+m2*d2^2+m2*L1^2+2*m2*L1*d2*cos(act_th(2,1))+I1+I2;
    M12 = m2*d2^2+m2*L1*d2*cos(act_th(2,1))+I2;
    M21 = m2*d2^2+m2*L1*d2*cos(act_th(2,1))+I2;
    M22 = m2*d2^2+I2;

    A112 = -2*m2*L1*d2*sin(act_th(2,i));
    A122 = -m2*L1*d2*sin(act_th(2,i));
    A211 = m2*L1*d2*sin(act_th(2,i));

    M = [M11 M12; M21 M22];
    C = [A112*act_th_dot(1,i)*act_th_dot(2,i)+A122*act_th_dot(2,i)*act_th_dot(2,i);
        A211*act_th_dot(1,i)*act_th_dot(1,i)];

    %T_act(1:2,i) = M*[theta_1_ddot(i) theta_2_ddot(i)]' + C + G;
    act_th_ddot(1:2,i) = inv(M)*(T_CoM(1:2,i)-C-G);
    act_th_dot(1:2,i+1) = act_th_dot(1:2,i) + del_t*act_th_ddot(1:2,i);
    act_th(1:2,i+1) = act_th(1:2,i) + del_t*act_th_dot(1:2,i);

    plot(0,0,'ko',CoM1(1),CoM1(2),'bo',CoM2(1),CoM2(2),'go',act_CoM_x(i),act_CoM_y(i),'ro');
    title('2-link planer manipulator CoM')
    p = line(Ax,Ay,'EraseMode','xor','LineWidth',[1]);
    axis([-0.1 0.7 -0.1 0.7]);

    drawnow;
end

f2 = figure()
i = 1:1:step_time;
plot(i,CoM_x,'.r', i,CoM_y,'.b', i,act_CoM_x(i),'.y', i,act_CoM_y(i),'.g');
title('CoM position(m)')
legend({'Ref CoM X', 'Ref CoM Y','Act CoM X','Act CoM Y'},'Location','northeast')

% comparison between two types of vector G
varNames = ["G1","G1_partial","G2","G2_partial"];
table(G1(i)', G1_partialdiff(i)', G2(i)', G2_partialdiff(i)','VariableNames',varNames)