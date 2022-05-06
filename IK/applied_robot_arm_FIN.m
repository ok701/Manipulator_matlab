%% 기본 setup
clear
clc
%%
% step마다 걸리는 시간
step_1_time = 0.7;
step_2_time = 1.5;
step_3_time = 2;
step_4_time = 3.5;

step_5_time = 3.5;
step_6_time = 2;

des_x = -0.1; % 도착 x위치

del_t = 0.005; % [sec] 0.001-sec sampling time

% 3개의 link
a1 = 0.250; a2 = 0.250; a3 = 0.150; % [m] link의 길이
m1 = 1; m2 = 0.5; m3 = 0.5; % link 질량 
lc1 = a1/2; lc2 = a2/2; lc3 = a3/2; % 무게중심까지의 거리
g = 9.81; 

reach_dis = 0.07;

% mass moment of inertia
I1 = (m1*a1^2)/12;
I2 = (m2*a2^2)/12;
I3 = (m3*a2^2)/12;

% animation setting
f1 = figure;
plot(0,0,'ko') 
axis([-0.7 0.7 -0.5 0.8]);
grid on

% 최초 EE 위치
theta0_1 = pi/4;
theta0_2 = pi/4+pi*3/2;
theta0_3 = pi/4+pi*3/2+pi*7/4;
x_0 = [0 a1*cos(theta0_1) a1*cos(theta0_1)+a2*cos(theta0_2) a1*cos(theta0_1)+a2*cos(theta0_2)+a3*cos(theta0_3)];
y_0 = [0 a1*sin(theta0_1) a1*sin(theta0_1)+a2*sin(theta0_2) a1*sin(theta0_1)+a2*sin(theta0_2)+a3*sin(theta0_3)];

p = line(x_0,y_0,'EraseMode','xor','LineWidth',(4));

%% 궤적 생성 

% 초기값
theta_1_dot(1) = 0; theta_1_ddot(1) = 0; theta_1_ddot(2) = 0;
theta_2_dot(1) = 0; theta_2_ddot(1) = 0; theta_2_ddot(2) = 0;
theta_3_dot(1) = 0; theta_3_ddot(1) = 0; theta_3_ddot(2) = 0;

count = 1;
for step = 1:1:6
    for i=1:1:(1/del_t) 
       
        % 1번째 Step
        if count == 1
            t = step_1_time*del_t*i; % 시간
            x(i) = x_0(4); % E.E의 x축 값
            y(i) = y_0(4)-reach_dis*0.5*(1-cos(pi/step_1_time*t)); % travling time = step_1_time
    
            theta_d = pi*3/2; % 유지하고 싶은 각도
            D = ((x(i)-a3*cos(theta_d))^2 + (y(i)-a3*sin(theta_d))^2 - a1^2 - a2^2)/(2*a1*a2);
            theta_2(i) = atan2(-sqrt(1-D^2),D);
            theta_1(i) = atan2( (y(i)-a3*sin(theta_d)),(x(i)-a3*cos(theta_d))) - atan2(a2*sin(theta_2(i)), a1+a2*cos(theta_2(i)) );
            theta_3(i) = 3/2*pi - (theta_1(i)+theta_2(i));

             if i == 1/del_t % 마지막일때
                f2 = figure;
                subplot(2,2,1);
                plot(x,y)
                title('Path Planning');
                xlabel('x(m)');
                ylabel('y(m)')


                subplot(2,2,2);
                plot(linspace(0,step_1_time,1/del_t),x,linspace(0,step_1_time,1/del_t),y)
                title('Trajectory');
                xlabel('time(s)');
                ylabel('displacement(m)')
                legend('x','y')
                

                subplot(2,2,[3,4])
                plot(linspace(0,step_1_time,1/del_t),rad2deg(theta_1), linspace(0,step_1_time,1/del_t),rad2deg(theta_2), linspace(0,step_1_time,1/del_t),rad2deg(theta_3));
                title('Joint');
                xlabel('time(s)');
                ylabel('angle(degree)')
                legend('theta1','theta2','theta3')

                sgtitle('Step 1')



                f8 = figure;
                subplot(2,2,1);
                x_axis = linspace(0,step_1_time,length(T(1,:))-2);
                y_axis = T(1,3:length(T(1,:)));
                plot(x_axis,y_axis)                
                hold on
                x_axis = linspace(0,step_1_time,length(T(1,:))-2);
                y_axis = T(2,3:length(T(1,:)));
                plot(x_axis,y_axis)
                hold on
                x_axis = linspace(0,step_1_time,length(T(1,:))-2);
                y_axis = T(3,3:length(T(1,:)));
                plot(x_axis,y_axis)
                
                title('Torque')
                xlabel('time(s)')
                ylabel('T(Nm)')  
                legend('1번 joint','2번 joint','3번 joint')
                

                subplot(2,2,2);
                x_axis = linspace(0,step_1_time,length(T(1,:))-2);
                y_axis = P(1,3:length(T(1,:)));
                plot(x_axis,y_axis)                
                hold on
                x_axis = linspace(0,step_1_time,length(T(1,:))-2);
                y_axis = P(2,3:length(T(1,:)));
                plot(x_axis,y_axis)
                hold on
                x_axis = linspace(0,step_1_time,length(T(1,:))-2);
                y_axis = P(3,3:length(T(1,:)));
                plot(x_axis,y_axis)
                
                title('Power')
                xlabel('time(s)')
                ylabel('P(Watt)')
                legend('1번 joint','2번 joint','3번 joint')
                
                subplot(2,2,[3,4]);
                plot(linspace(0,step_1_time,1/del_t-1),rad2deg(theta_1_dot), linspace(0,step_1_time,1/del_t-1),rad2deg(theta_2_dot), linspace(0,step_1_time,1/del_t-1),rad2deg(theta_3_dot))
                
                title('Angular Velocity')
                xlabel('time(s)')
                ylabel('V(deg/s)')
                legend('1번 joint','2번 joint','3번 joint')

                sgtitle('Step 1')

            end
        end
        
        % 2번째 Step
        if count == 2
            t = step_2_time*del_t*i; % 시간
            x(i) = x_0(4);
            y(i) = y_0(4)-reach_dis+(reach_dis)*0.5*(1-cos(pi/step_2_time*t));
    
            theta_d = pi*3/2;
            D = ((x(i)-a3*cos(theta_d))^2 + (y(i)-a3*sin(theta_d))^2 - a1^2 - a2^2)/(2*a1*a2);
            theta_2(i) = atan2(-sqrt(1-D^2),D);
            theta_1(i) = atan2((y(i)-a3*sin(theta_d)),(x(i)-a3*cos(theta_d))) - atan2(a2*sin(theta_2(i)), a1+a2*cos(theta_2(i)));
            theta_3(i) = 3/2*pi - (theta_1(i)+theta_2(i));

            if i == 1/del_t % 마지막일때
                step3_init_x = x(i); % 다음 step을 위해 마지막 위치 저장
                step3_init_y = y(i);

                f3 = figure;
                subplot(2,2,1);
                plot(x,y)
                title('Path Planning');
                xlabel('x(m)');
                ylabel('y(m)')


                subplot(2,2,2);
                plot(linspace(0,step_2_time,1/del_t),x,linspace(0,step_2_time,1/del_t),y)
                title('Trajectory');
                xlabel('time(s)');
                ylabel('displacement(m)')
                legend('x','y')
                

                subplot(2,2,[3,4])
                plot(linspace(0,step_2_time,1/del_t),rad2deg(theta_1), linspace(0,step_2_time,1/del_t),rad2deg(theta_2), linspace(0,step_2_time,1/del_t),rad2deg(theta_3))
                title('Joint');
                xlabel('time(s)');
                ylabel('angle(degree)')
                legend('theta1','theta2','theta3')

                sgtitle('Step 2')

                f9 = figure;
                subplot(2,2,1);
                x_axis = linspace(0,step_2_time,length(T(1,:))-2);
                y_axis = T(1,1:length(T(1,:))-2);
                plot(x_axis,y_axis)                
                hold on
                x_axis = linspace(0,step_2_time,length(T(1,:))-2);
                y_axis = T(2,1:length(T(1,:))-2);
                plot(x_axis,y_axis)
                hold on
                x_axis = linspace(0,step_2_time,length(T(1,:))-2);
                y_axis = T(3,1:length(T(1,:))-2);
                plot(x_axis,y_axis)
                
                title('Torque')
                xlabel('time(s)')
                ylabel('T(Nm)')  
                legend('1번 joint','2번 joint','3번 joint')
                

                subplot(2,2,2);
                x_axis = linspace(0,step_2_time,length(T(1,:))-2);
                y_axis = P(1,3:length(T(1,:)));
                plot(x_axis,y_axis)                
                hold on
                x_axis = linspace(0,step_2_time,length(T(1,:))-2);
                y_axis = P(2,3:length(T(1,:)));
                plot(x_axis,y_axis)
                hold on
                x_axis = linspace(0,step_2_time,length(T(1,:))-2);
                y_axis = P(3,3:length(T(1,:)));
                plot(x_axis,y_axis)
                
                title('Power')
                xlabel('time(s)')
                ylabel('P(Watt)')
                legend('1번 joint','2번 joint','3번 joint')

                subplot(2,2,[3,4]);
                plot(linspace(0,step_2_time,1/del_t-1),rad2deg(theta_1_dot(1,1:1/del_t-1)),linspace(0,step_2_time,1/del_t-1),rad2deg(theta_2_dot(1,1:1/del_t-1)),linspace(0,step_2_time,1/del_t-1),rad2deg(theta_3_dot(1,1:1/del_t-1)))
                
                title('Angular Velocity')
                xlabel('time(s)')
                ylabel('V(deg/s)')
                legend('1번 joint','2번 joint','3번 joint')

                sgtitle('Step 2')

            end
        end

        % 3번째 Step
        if count == 3
            t = step_3_time*del_t*i; % 시간

            x(i) = step3_init_x-( step3_init_x-sqrt(a2^2-(a1-a3)^2) )*0.5*(1-cos(pi/step_3_time*t));
            y(i) = ( step3_init_y/(step3_init_x-sqrt(a2^2-(a1-a3)^2)) )*( x(i)-sqrt(a2^2-(a1-a3)^2) );
    
            theta_d = pi*3/2;
            D = ((x(i)-a3*cos(theta_d))^2 + (y(i)-a3*sin(theta_d))^2 - a1^2 - a2^2)/(2*a1*a2);
            theta_2(i) = atan2(-sqrt(1-D^2),D);
            theta_1(i) = atan2((y(i)-a3*sin(theta_d)),(x(i)-a3*cos(theta_d))) - atan2(a2*sin(theta_2(i)), a1+a2*cos(theta_2(i)));
            theta_3(i) = 3/2*pi - (theta_1(i)+theta_2(i));

            if i == 1/del_t % 마지막일때
                step4_init_x = x(i); % 다음 step을 위해 마지막 위치 저장
                step4_init_y = y(i);
                step4_init_theta_1 = theta_1(i); % 다음 step을 위해 마지막 위치 저장
                step4_init_theta_2 = theta_2(i);
                step4_init_theta_3 = theta_3(i);

                f4 = figure;
                subplot(2,2,1);
                plot(x,y)
                title('Path Planning');
                xlabel('x(m)');
                ylabel('y(m)')


                subplot(2,2,2);
                plot(linspace(0,step_3_time,1/del_t),x,linspace(0,step_3_time,1/del_t),y)
                title('Trajectory');
                xlabel('time(s)');
                ylabel('displacement(m)')
                legend('x','y')
                

                subplot(2,2,[3,4])
                plot(linspace(0,step_3_time,1/del_t),rad2deg(theta_1), linspace(0,step_3_time,1/del_t),rad2deg(theta_2), linspace(0,step_3_time,1/del_t),rad2deg(theta_3))
                title('Joint');
                xlabel('time(s)');
                ylabel('angle(degree)')
                legend('theta1','theta2','theta3')

                sgtitle('Step 3')

                f10 = figure;
               subplot(2,2,1);
                x_axis = linspace(0,step_3_time,length(T(1,:))-2);
                y_axis = T(1,1:length(T(1,:))-2);
                plot(x_axis,y_axis)                
                hold on
                x_axis = linspace(0,step_3_time,length(T(1,:))-2);
                y_axis = T(2,1:length(T(1,:))-2);
                plot(x_axis,y_axis)
                hold on
                x_axis = linspace(0,step_3_time,length(T(1,:))-2);
                y_axis = T(3,1:length(T(1,:))-2);
                plot(x_axis,y_axis)
                
                title('Torque')
                xlabel('time(s)')
                ylabel('T(Nm)')  
                legend('1번 joint','2번 joint','3번 joint')
                

                subplot(2,2,2);
                x_axis = linspace(0,step_3_time,length(T(1,:))-2);
                y_axis = P(1,3:length(T(1,:)));
                plot(x_axis,y_axis)                
                hold on
                x_axis = linspace(0,step_3_time,length(T(1,:))-2);
                y_axis = P(2,3:length(T(1,:)));
                plot(x_axis,y_axis)
                hold on
                x_axis = linspace(0,step_3_time,length(T(1,:))-2);
                y_axis = P(3,3:length(T(1,:)));
                plot(x_axis,y_axis)
                
                title('Power')
                xlabel('time(s)')
                ylabel('P(Watt)')
                legend('1번 joint','2번 joint','3번 joint')

                subplot(2,2,[3,4]);
                plot(linspace(0,step_3_time,1/del_t-1),rad2deg(theta_1_dot(1,1:1/del_t-1)),linspace(0,step_3_time,1/del_t-1),rad2deg(theta_2_dot(1,1:1/del_t-1)),linspace(0,step_3_time,1/del_t-1),rad2deg(theta_3_dot(1,1:1/del_t-1)))
                
                title('Angular Velocity')
                xlabel('time(s)')
                ylabel('V(deg/s)')
                legend('1번 joint','2번 joint','3번 joint')


                sgtitle('Step 3')

            end
        end

        % 4번째 Step
        if count == 4
            t = step_4_time*del_t*i; % 시간 
            theta_1(i) = step4_init_theta_1;
            theta_2(i) = step4_init_theta_2+deg2rad(170)*0.5*(1-cos(pi/step_4_time*t));
            theta_3(i) = step4_init_theta_3; 

            x(i) = Ax(4);
            y(i) = Ay(4);

             if i == 1/del_t % 마지막일때
                step5_init_theta_1 = theta_1(i); % 다음 step을 위해 마지막 위치 저장
                step5_init_theta_2 = theta_2(i);
                step5_init_theta_3 = theta_3(i);

                f5 = figure;
                subplot(2,2,1);
                plot(x,y)
                title('Path Planning');
                xlabel('x(m)');
                ylabel('y(m)')


                subplot(2,2,2);
                plot(linspace(0,step_4_time,1/del_t),x,linspace(0,step_4_time,1/del_t),y)
                title('Trajectory');
                xlabel('time(s)');
                ylabel('displacement(m)')
                legend('x','y')
                

                subplot(2,2,[3,4])
                plot(linspace(0,step_4_time,1/del_t),rad2deg(theta_1), linspace(0,step_4_time,1/del_t),rad2deg(theta_2), linspace(0,step_4_time,1/del_t),rad2deg(theta_3))
                title('Joint');
                xlabel('time(s)');
                ylabel('angle(degree)')
                legend('theta1','theta2','theta3')

                sgtitle('Step 4')

                                f11 = figure;
  subplot(2,2,1);
                x_axis = linspace(0,step_4_time,length(T(1,:))-2);
                y_axis = T(1,1:length(T(1,:))-2);
                plot(x_axis,y_axis)                
                hold on
                x_axis = linspace(0,step_4_time,length(T(1,:))-2);
                y_axis = T(2,1:length(T(1,:))-2);
                plot(x_axis,y_axis)
                hold on
                x_axis = linspace(0,step_4_time,length(T(1,:))-2);
                y_axis = T(3,1:length(T(1,:))-2);
                plot(x_axis,y_axis)
                
                title('Torque')
                xlabel('time(s)')
                ylabel('T(Nm)')  
                legend('1번 joint','2번 joint','3번 joint')
                

                subplot(2,2,2);
                x_axis = linspace(0,step_4_time,length(T(1,:))-2);
                y_axis = P(1,3:length(T(1,:)));
                plot(x_axis,y_axis)                
                hold on
                x_axis = linspace(0,step_4_time,length(T(1,:))-2);
                y_axis = P(2,3:length(T(1,:)));
                plot(x_axis,y_axis)
                hold on
                x_axis = linspace(0,step_4_time,length(T(1,:))-2);
                y_axis = P(3,3:length(T(1,:)));
                plot(x_axis,y_axis)
                
                title('Power')
                xlabel('time(s)')
                ylabel('P(Watt)')
                legend('1번 joint','2번 joint','3번 joint')

                subplot(2,2,[3,4]);
                plot(linspace(0,step_4_time,1/del_t-1),rad2deg(theta_1_dot(1,1:1/del_t-1)),linspace(0,step_4_time,1/del_t-1),rad2deg(theta_2_dot(1,1:1/del_t-1)),linspace(0,step_4_time,1/del_t-1),rad2deg(theta_3_dot(1,1:1/del_t-1)))
                
                title('Angular Velocity')
                xlabel('time(s)')
                ylabel('V(deg/s)')
                legend('1번 joint','2번 joint','3번 joint')

                sgtitle('Step 4')

            end
        end
    
        % 복귀
        % 5번째 Step
        if count == 5
            t = step_5_time*del_t*i; % 시간 
            theta_1(i) = step5_init_theta_1;
            theta_2(i) = step5_init_theta_2-deg2rad(170)*0.5*(1-cos(pi/step_5_time*t));
            theta_3(i) = step5_init_theta_3; 
       
            x(i) = Ax(4);
            y(i) = Ay(4);
        
         if i == 1/del_t % 마지막일때
                step5_init_theta_1 = theta_1(i); % 다음 step을 위해 마지막 위치 저장
                step5_init_theta_2 = theta_2(i);
                step5_init_theta_3 = theta_3(i);
        
                f6 = figure;
                subplot(2,2,1);
                plot(x,y)
                title('Path Planning');
                xlabel('x(m)');
                ylabel('y(m)')


                subplot(2,2,2);
                plot(linspace(0,step_5_time,1/del_t),x,linspace(0,step_5_time,1/del_t),y)
                title('Trajectory');
                xlabel('time(s)');
                ylabel('displacement(m)')
                legend('x','y')
                

                subplot(2,2,[3,4])
                plot(linspace(0,step_5_time,1/del_t),rad2deg(theta_1), linspace(0,step_5_time,1/del_t),rad2deg(theta_2), linspace(0,step_5_time,1/del_t),rad2deg(theta_3))
                title('Joint');
                xlabel('time(s)');
                ylabel('angle(degree)')
                legend('theta1','theta2','theta3')

                sgtitle('Step 5')

                                f12 = figure;
 subplot(2,2,1);
                x_axis = linspace(0,step_5_time,length(T(1,:))-2);
                y_axis = T(1,1:length(T(1,:))-2);
                plot(x_axis,y_axis)                
                hold on
                x_axis = linspace(0,step_5_time,length(T(1,:))-2);
                y_axis = T(2,1:length(T(1,:))-2);
                plot(x_axis,y_axis)
                hold on
                x_axis = linspace(0,step_5_time,length(T(1,:))-2);
                y_axis = T(3,1:length(T(1,:))-2);
                plot(x_axis,y_axis)
                
                title('Torque')
                xlabel('time(s)')
                ylabel('T(Nm)')  
                legend('1번 joint','2번 joint','3번 joint')
                

                subplot(2,2,2);
                x_axis = linspace(0,step_5_time,length(T(1,:))-2);
                y_axis = P(1,3:length(T(1,:)));
                plot(x_axis,y_axis)                
                hold on
                x_axis = linspace(0,step_5_time,length(T(1,:))-2);
                y_axis = P(2,3:length(T(1,:)));
                plot(x_axis,y_axis)
                hold on
                x_axis = linspace(0,step_5_time,length(T(1,:))-2);
                y_axis = P(3,3:length(T(1,:)));
                plot(x_axis,y_axis)
                
                title('Power')
                xlabel('time(s)')
                ylabel('P(Watt)')
                legend('1번 joint','2번 joint','3번 joint')

                subplot(2,2,[3,4]);
                plot(linspace(0,step_5_time,1/del_t-1),rad2deg(theta_1_dot(1,1:1/del_t-1)),linspace(0,step_5_time,1/del_t-1),rad2deg(theta_2_dot(1,1:1/del_t-1)),linspace(0,step_5_time,1/del_t-1),rad2deg(theta_3_dot(1,1:1/del_t-1)))
                
                title('Angular Velocity')
                xlabel('time(s)')
                ylabel('V(deg/s)')
                legend('1번 joint','2번 joint','3번 joint')


                sgtitle('Step 5')

        end
        
        
        end

        % 6번째 Step
        if count == 6
            t = step_6_time*del_t*i; % 시간

            x(i) = step4_init_x+( step3_init_x-sqrt(a2^2-(a1-a3)^2) )*0.5*(1-cos(pi/step_6_time*t));
            y(i) = ( step3_init_y/(step3_init_x-sqrt(a2^2-(a1-a3)^2)) )*( x(i)-sqrt(a2^2-(a1-a3)^2) );
    
            theta_d = pi*3/2;
            D = ((x(i)-a3*cos(theta_d))^2 + (y(i)-a3*sin(theta_d))^2 - a1^2 - a2^2)/(2*a1*a2);
            theta_2(i) = atan2(-sqrt(1-D^2),D);
            theta_1(i) = atan2((y(i)-a3*sin(theta_d)),(x(i)-a3*cos(theta_d))) - atan2(a2*sin(theta_2(i)), a1+a2*cos(theta_2(i)));
            theta_3(i) = 3/2*pi - (theta_1(i)+theta_2(i));

            if i == 1/del_t % 마지막일때
                f7 = figure;
                subplot(2,2,1);
                plot(x,y)
                title('Path Planning');
                xlabel('x(m)');
                ylabel('y(m)')


                subplot(2,2,2);
                plot(linspace(0,step_6_time,1/del_t),x,linspace(0,step_6_time,1/del_t),y)
                title('Trajectory');
                xlabel('time(s)');
                ylabel('displacement(m)')
                legend('x','y')
                

                subplot(2,2,[3,4])
                plot(linspace(0,step_6_time,1/del_t),rad2deg(theta_1), linspace(0,step_6_time,1/del_t),rad2deg(theta_2), linspace(0,step_6_time,1/del_t),rad2deg(theta_3))
                title('Joint');
                xlabel('time(s)');
                ylabel('angle(degree)')
                legend('theta1','theta2','theta3')

                sgtitle('Step 6')

                                f13 = figure;
 subplot(2,2,1);
                x_axis = linspace(0,step_6_time,length(T(1,:))-2);
                y_axis = T(1,1:length(T(1,:))-2);
                plot(x_axis,y_axis)                
                hold on
                x_axis = linspace(0,step_6_time,length(T(1,:))-2);
                y_axis = T(2,1:length(T(1,:))-2);
                plot(x_axis,y_axis)
                hold on
                x_axis = linspace(0,step_6_time,length(T(1,:))-2);
                y_axis = T(3,1:length(T(1,:))-2);
                plot(x_axis,y_axis)
                
                title('Torque')
                xlabel('time(s)')
                ylabel('T(Nm)')  
                legend('1번 joint','2번 joint','3번 joint')
                

                subplot(2,2,2);
                x_axis = linspace(0,step_6_time,length(T(1,:))-2);
                y_axis = P(1,3:length(T(1,:)));
                plot(x_axis,y_axis)                
                hold on
                x_axis = linspace(0,step_6_time,length(T(1,:))-2);
                y_axis = P(2,3:length(T(1,:)));
                plot(x_axis,y_axis)
                hold on
                x_axis = linspace(0,step_6_time,length(T(1,:))-2);
                y_axis = P(3,3:length(T(1,:)));
                plot(x_axis,y_axis)
                
                title('Power')
                xlabel('time(s)')
                ylabel('P(Watt)')
                legend('1번 joint','2번 joint','3번 joint')

                subplot(2,2,[3,4]);
                plot(linspace(0,step_6_time,1/del_t-1),rad2deg(theta_1_dot(1,1:1/del_t-1)),linspace(0,step_6_time,1/del_t-1),rad2deg(theta_2_dot(1,1:1/del_t-1)),linspace(0,step_6_time,1/del_t-1),rad2deg(theta_3_dot(1,1:1/del_t-1)))
                
                title('Angular Velocity')
                xlabel('time(s)')
                ylabel('V(deg/s)')
                legend('1번 joint','2번 joint','3번 joint')


                sgtitle('Step 6')

            end
        end
  
    
    % theta_1
    if i >= 2
        theta_1_dot(i) = (theta_1(i) - theta_1(i-1))/del_t;
    end
    
    if i >= 3
        theta_1_ddot(i) = (theta_1_dot(i) - theta_1_dot(i-1))/del_t;
    end
    
    % theta_2
    if i >= 2
        theta_2_dot(i) = (theta_2(i) - theta_2(i-1))/del_t;
    end
    
    if i >= 3
        theta_2_ddot(i) = (theta_2_dot(i) - theta_2_dot(i-1))/del_t;
    end
    
    % theta_3
    if i >= 2
        theta_3_dot(i) = (theta_3(i) - theta_3(i-1))/del_t;
    end
    
    if i >= 3
        theta_3_ddot(i) = (theta_3_dot(i) - theta_3_dot(i-1))/del_t;
    end
    
    Ax = [0 a1*cos(theta_1(i)) a1*cos(theta_1(i))+a2*cos(theta_1(i)+theta_2(i)) a1*cos(theta_1(i))+a2*cos(theta_1(i)+theta_2(i))+a3*cos(theta_1(i)+theta_2(i)+theta_3(i))];
    Ay = [0 a1*sin(theta_1(i)) a1*sin(theta_1(i))+a2*sin(theta_1(i)+theta_2(i)) a1*sin(theta_1(i))+a2*sin(theta_1(i)+theta_2(i))+a3*sin(theta_1(i)+theta_2(i)+theta_3(i))];
    
    set(p,'XData',Ax,'YData',Ay);
    
    drawnow;

    % 여기서부터 F,v,P 계산
    C1 = cos(theta_1(i));
    C2 = cos(theta_2(i));
    C3 = cos(theta_3(i));
    C12 = cos(theta_1(i)+theta_2(i));
    C13 = cos(theta_1(i)+theta_3(i));
    C23 = cos(theta_2(i)+theta_3(i));
    C123 = cos(theta_1(i)+theta_2(i)+theta_3(i));
    S1 = sin(theta_1(i));
    S2 = sin(theta_2(i));
    S3 = sin(theta_3(i));
    S12 = sin(theta_1(i)+theta_2(i));
    S13 = sin(theta_1(i)+theta_3(i));
    S23 = sin(theta_2(i)+theta_3(i));
    
    % Inertia Matrix
    M11 = m1*lc1^2 + I1 + m2*a1^2 + m2*lc2^2 + 2*m2*a1*lc2*C2 + I2 + m3*a1^2 + m3*a2^2 + m3*lc3^2 + m3*a1*a2*2*C2 + m3*a2*lc3*2*C3 + m3*a1*lc3*2*C23 + I3;
    M12 = m2*lc2^2 + m2*a1*lc2*C2 +I2 + m3*a2^2 + m3*lc3^2 + m3*a1*a2*C2 + m3*a2*lc3*2*C3 + m3*a1*lc3*C23 + I3;  
    M13 = m3*lc3^2 + m3*a2*lc3*C3 + m3*a1*lc3*C23 + I3; 
    M21 = m2*lc2^2 + I3 + m2*a1*lc2*C2 + I2 + m3*a2^2 + m3*lc3^2 + m3*a1*a2*C2 + 2*m3*a2*lc3*C3 + m3*a1*lc3*C23;
    M22 = m2*lc2^2 + I3 + I2 + m3*a2^2 + m3*lc3^2 + 2*m3*a2*lc3*C3;
    M23 = m3*lc3^2 + m3*a2*lc3*C3 + I3;
    M31 = m3*lc3^2 + m3*a2*lc3*C3 + m3*a1*lc3*C23 + I3;
    M32 = m3*lc3^2 + m3*a2*lc3*C3 + I3;
    M33 = m3*lc3^2 + I3;
    
    % Centrifugal and Coloilis Matrix
    A112 = -m2*a1*lc2*2*S2 - m3*a1*a2*2*S2 - 2*m3*a1*lc3*S23;
    A123 = -m3*a2*lc3*2*S3 - 2*m3*a1*lc3*S23;
    A113 = -2*m3*a2*lc3*S3 - 2*m3*a1*lc3*S23;
    A122 = -m2*a1*lc3*S3 - m3*a1*a2*S2 - m3*a1*lc3*S23;
    A133 = -m3*a1*lc3*S23 - m3*a2*lc3*S3;
    A211 = m2*a1*lc2*S2 + m3*a1*a2*S2 + m3*a1*lc3*S23;
    A233 = -m3*a2*lc3*S3;
    A212 = 0;
    A223 = -m3*a2*lc3*2*S3;
    A213 = -2*m3*a2*lc3*S3;
    A311 = m3*a2*lc3*S3 + m3*a1*lc3*S23;
    A322 = m3*a2*lc3*S3;
    A312 = 2*m3*a2*lc3*S3;
    A323 = 0;
    A313 = 0;
    
    dot12 = theta_1_dot(i)*theta_2_dot(i);
    dot13 = theta_1_dot(i)*theta_3_dot(i);
    dot23 = theta_2_dot(i)*theta_3_dot(i);
    dot1sq = (theta_1_dot(i))^2;
    dot2sq = (theta_2_dot(i))^2;
    dot3sq = (theta_3_dot(i))^2;
    
    % Gravity
    G1 = m1*g*lc1*C1 + m2*g*(a1*C1+lc2*C12) + m3*g*(a1*C1 + a2*C12 + lc3*C123);
    G2 = m2*g*lc2*C12 + m3*g*(a2*C12 + lc3*C123);
    G3 = m3*g*lc3*C123;
    
    
    M = [M11,M12,M13;M21,M22,M23;M31,M32,M33];
    C = [A112*dot12+A123*dot23+A113*dot13+A133*dot3sq+A122*dot2sq;A211*dot1sq+A233*dot3sq^2+A212*dot12+A223*dot23+A213*dot13;A311*dot1sq+A322*dot2sq+A312*dot12+A323*dot23+A313*dot13];
    G = [G1; G2; G3];
    
    T(1:3,i) = M*[theta_1_ddot(i) theta_2_ddot(i) theta_3_ddot(i)]' + C + G; 
    P(1:3,i) = abs([theta_1_dot(i) theta_2_dot(i) theta_3_dot(i)]'.*T(1:3,i));

    end
    count = count + 1;
end
