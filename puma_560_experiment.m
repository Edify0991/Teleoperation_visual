%% puma560实验，从桌面一移动到桌面二
close all;
clc;
mdl_puma560;
deg = pi/180;
figure
theta = zeros(1,6);
p560.plot(theta);
hold on;
draw_box([0.3;-0.265;-0.5],[0.7;0.265;-0.5],'y','mesh','k','alpha',0.3);%画放货平台
draw_box([-0.365;0.3;-0.5],[0.1653;0.7;-0.5],'y','mesh','k','alpha',0.3);%画放货平台
[x,y,z] = ellipsoid(0.5,0,-0.4,0.05,0.05,0.05);P_1 = surf(x,y,z);%球1
[x,y,z] = ellipsoid(0.5,0.15,-0.4,0.05,0.05,0.05);P_3 = surf(x,y,z);%球2
pause(5);

t=[0:0.1:4];%4秒完成轨迹，步长0.1

%获取各个位置的位姿方程
T_00 = p560.fkine(theta);
T_0 = transl(0.5,0,0);
T_st = transl(0.5,0,-0.4);
T_1 = transl(-0.1,0.5,-0.4);
T_f = transl(-0.1,0.5,0.1);

%第一组运动
q1 = p560.ikine(T_00,'mask',[1 1 1 1 1 1]); %如果是[1 1 1 1 1 0]，则最后一个关节角度一直是0
q2 = p560.ikine(T_0,'mask',[1 1 1 1 1 1],'q0',q1); 
[q,qt,qtt]=jtraj(q1,q2,t);
p560.plot(q)%动态绘制轨迹运动
%%第二组运动
q3 = p560.ikine(T_0,'mask',[1 1 1 1 1 1]); %如果是[1 1 1 1 1 0]，则最后一个关节角度一直是0
q4 = p560.ikine(T_st,'mask',[1 1 1 1 1 1],'q0',q3); 
[q,qt,qtt]=jtraj(q3,q4,t);

p560.plot(q);%动态绘制轨迹运动

%%第三组运动
delete (P_1);
t = [0:0.1:6];
q5 = p560.ikine(T_st,'mask',[1 1 1 1 1 1]); %如果是[1 1 1 1 1 0]，则最后一个关节角度一直是0
q6 = p560.ikine(T_1,'mask',[1 1 1 1 1 1],'q0',q5); 
[q,qt,qtt]=jtraj(q5,q6,t);
p560.plot(q,'trail','r');%动态绘制轨迹运动
[x,y,z] = ellipsoid(-0.1,0.5,-0.4,0.05,0.05,0.05);P_2 = surf(x,y,z);

 %%第四组运动
q7 = p560.ikine(T_1,'mask',[1 1 1 1 1 1]); %如果是[1 1 1 1 1 0]，则最后一个关节角度一直是0
q8 = p560.ikine(T_f,'mask',[1 1 1 1 1 1],'q0',q7); 
[q,qt,qtt]=jtraj(q7,q8,t);
p560.plot(q)%动态绘制轨迹运动
%%复位动作
q9 = p560.ikine(T_f,'mask',[1 1 1 1 1 1]); %如果是[1 1 1 1 1 0]，则最后一个关节角度一直是0
q10 = p560.ikine(T_00,'mask',[1 1 1 1 1 1],'q0',q9); 
[q,qt,qtt]=jtraj(q9,q10,t);
p560.plot(q,'trail','b-','movie','kk_1.gif')%动态绘制轨迹运动,绘制GIF动图，并导出

%% 抓取第二个球
% Theta_21 = [33.3 5.6 -174 0 0 0]*deg;
% Theta_22 = [33.3 45 -213 0 0 0]*deg;
% Theta_23 = [133.3 45 -213 0 0 0]*deg;
% Theta_24 = [133.3 5.6 -180 0 0 0]*deg;
% Theta_25 = [0 0 0 0 0 0];
% 
% T0 = p560.fkine(Theta_21);
% T1 = p560.fkine(Theta_22);
% T2 = p560.fkine(Theta_23);
% T3 = p560.fkine(Theta_24);
% T4 = p560.fkine(Theta_25);

T_1 = transl(0.5,0.15,-0.45);
T_2 = transl(-0.25,0.45,-0.45);
t=[0:0.1:4];%8秒完成轨迹，步长0.1

q_1 = p560.ikine(T_00,'mask',[1 1 1 1 1 1]); %如果是[1 1 1 1 1 0]，则最后一个关节角度一直是0
q_2 = p560.ikine(T_1,'mask',[1 1 1 1 1 1],'q0',q_1); 
[q,qt,qtt]=jtraj(q_1,q_2,t);
p560.plot(q)%动态绘制轨迹运动

delete(P_3);
q_3 = p560.ikine(T_1,'mask',[1 1 1 1 1 1]); %如果是[1 1 1 1 1 0]，则最后一个关节角度一直是0
q_4 = p560.ikine(T_2,'mask',[1 1 1 1 1 1],'q0',q_3); 
[q,qt,qtt]=jtraj(q_3,q_4,t);
p560.plot(q)%动态绘制轨迹运动
[x,y,z] = ellipsoid(-0.25,0.45,-0.45,0.05,0.05,0.05);P_4 = surf(x,y,z);%第二个球移动后的位置

q_5 = p560.ikine(T_2,'mask',[1 1 1 1 1 1]); %如果是[1 1 1 1 1 0]，则最后一个关节角度一直是0
q_6 = p560.ikine(T_00,'mask',[1 1 1 1 1 1],'q0',q_5); 
[q,qt,qtt]=jtraj(q_5,q_6,t);
p560.plot(q)%动态绘制轨迹运动
