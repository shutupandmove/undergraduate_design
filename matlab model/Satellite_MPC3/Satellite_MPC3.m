%%
%清屏
clear all;
clc;
%%
% 容忍的控制范围
% 在1000km轨道上，按正弦函数计算，100m对应5.73e-3deg，20m对应1.15e-3deg,10m对应5.73e-4deg，5m对应2.86e-4deg
% 距离上选取10m作为极限
% i 选取5.73e-3deg≈1e-04rads作为极限
% e 选取5e-05作为极限
% w 暂且0.0005
% 重点控制a e i三项，对M不做控制

% RTN坐标轴方向上允许的最大推力为100mN
% 换算成加速度约为0.0037m/s^2
% u_lim = 0.0037;
u_lim = inf;

G = 6.67e-11;%重力常数，m^3/(kg·s^2)
m_e = 5.965e24;%地球质量，kg
meu = G*m_e;

% T = -131.21+0.00299*r;近地点大气温度，来自NASA大气模型
% p = 2.488*[(T+273.1)/216.6]^(-11.388);%近地点大气压强
% atm_dens_p = p/[0.2869*(T+273.1)];%大气密度kg/m^3
% atm_dens_p = 2.7544e-16;%近地点大气密度kg/m^3

% syms h;
% r(h) = [2.488*[(-131.21+0.00299*h+273.1)/216.6]^(-11.388)]/[0.2869*(-131.21+0.00299*h+273.1)];
% beta = -(dr/dh/r);
% beta = 1.2296e-05;%大气标高的倒数m^-1

n_s = 13.72814543*2*pi()/24/60/60;%真近点角角速率均值,rads/s
R_e = 6378.2e3;%地球赤道半径km

J2 = 1082.63e-6;%地球引力场二阶带谐系数
s = 0.2*0.2;%受力面积m^2,8U
Cd = 2.2;%气动系数,一般取2.2
m_s = 27;%卫星质量(算上预计的推进系统)

K = 2;%反射系数
p_solar = 4.65e-06;%太阳光压强度，N/m^2

a_ini = (960e3+1019e3)/2+R_e;%m
e_ini = 0.0039866;
i_ini = 82.9236/180*pi();%82.9236°
w_ini = 297.2910/180*pi();%近地点辐角297.2910°
Ohm_ini = 272.9481/180*pi();%升交点赤经272.9481°
theta_ini = 0;
theta1_ini = 0;
theta2_ini = 1e-7;
theta3_ini = 2e-7;
%%
%编队距离
Form_dis = 1e3;
%编队控制器参数
kp = 4e-7;
ki = 0;
kd = 0;
%%
figure;

x = [out.yout{9}.Values.Data(:,1), out.yout{17}.Values.Data(:,1), out.yout{25}.Values.Data(:,1)];
y = [out.yout{9}.Values.Data(:,2), out.yout{17}.Values.Data(:,2), out.yout{25}.Values.Data(:,2)];
z = [out.yout{9}.Values.Data(:,3), out.yout{17}.Values.Data(:,3), out.yout{25}.Values.Data(:,3)];
plot3(x,y,z);

[vol,t] = [size(out.yout{9}.Values.Data)];

plot3(out.yout{9}.Values.Data(1,1),out.yout{9}.Values.Data(1,2),out.yout{9}.Values.Data(1,3),'g');
plot3(out.yout{17}.Values.Data(1,1),out.yout{17}.Values.Data(1,2),out.yout{17}.Values.Data(1,3),'g');
plot3(out.yout{25}.Values.Data(1,1),out.yout{25}.Values.Data(1,2),out.yout{25}.Values.Data(1,3),'g');

mid = round(vol/2);
plot3(out.yout{9}.Values.Data(mid,1),out.yout{9}.Values.Data(mid,2),out.yout{9}.Values.Data(mid,3),'m');
plot3(out.yout{17}.Values.Data(mid,1),out.yout{17}.Values.Data(mid,2),out.yout{17}.Values.Data(mid,3),'m');
plot3(out.yout{25}.Values.Data(mid,1),out.yout{25}.Values.Data(mid,2),out.yout{25}.Values.Data(mid,3),'m');

ed = vol;
plot3(out.yout{9}.Values.Data(ed,1),out.yout{9}.Values.Data(ed,2),out.yout{9}.Values.Data(ed,3),'k');
plot3(out.yout{17}.Values.Data(ed,1),out.yout{17}.Values.Data(ed,2),out.yout{17}.Values.Data(ed,3),'k');
plot3(out.yout{25}.Values.Data(ed,1),out.yout{25}.Values.Data(ed,2),out.yout{25}.Values.Data(ed,3),'k');

grid on;
% plot3(out.yout{9}.Values.Data(:,1),out.yout{9}.Values.Data(:,2),out.yout{9}.Values.Data(:,3));
% plot3(out.yout{17}.Values.Data(:,1),out.yout{17}.Values.Data(:,2),out.yout{17}.Values.Data(:,3));
% plot3(out.yout{25}.Values.Data(:,1),out.yout{25}.Values.Data(:,2),out.yout{25}.Values.Data(:,3));