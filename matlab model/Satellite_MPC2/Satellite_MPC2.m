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
%%
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
a = a_ini;
e = e_ini;
i = i_ini;
w = w_ini;
Ohm = Ohm_ini;
theta = theta_ini;
%%
%绘图
figure;
hold on;
plot(STKPERFINAL(:,1),STKPERFINAL(:,2),'LineWidth',1.5);
% plot(out.tout, out.yout{5}.Values.Data(:,1),'LineWidth',1.5);
% plot(STKPERFINAL(:,1),STKPERFINAL(:,5),STKPERFINAL(:,1),STKPERFINAL(:,6),STKPERFINAL(:,1),STKPERFINAL(:,7),'LineWidth',1.5);
grid;
xlabel('t/s');
ylabel('a/km');
set(gca,'XLim',[0,604801]);
set(gca,'FontSize',30);
hold off;
% legend('Ω','w','θ');
%%
%变量初始化

%误差
da = 500;
de = -0.0001;
di = 0.005/180*pi();
dOhm = 0;
dw = 0;
dtheta = 0;

% a = a_ini-da;
% e = e_ini-de;
% i = i_ini-di;
% w = w_ini-dw;
% Ohm = Ohm_ini-dOhm;
% theta = theta_ini-dtheta;

% p_s = a*(1-e^2);%轨道半通径
% h = sqrt(meu*p_s);%轨道角动量幅值
% b = a*sqrt(1-e^2);%半短轴
% r = a*(1-e^2)/(1+e*cos(theta));%地心距
% atm_dens_p = [2.488*(((-131.21+0.00299*r)+273.1)/216.6)^(-11.388)]/[0.2869*((-131.21+0.00299*r)+273.1)];

%非被控变量
% r = 960000;%m,地心距初始值
%%
%MPC控制器的初始化

%采样时间
Ts = 0.001;%s

%step数量
k_steps=60/Ts;%1min
% k_steps=3600/Ts;%1h
% k_steps=86400/Ts;%1d
% k_steps=604800/Ts;%1week

%二次规划系数矩阵
%    [da,de,di,dOhm,dw,dM_s]
Q = diag([1,1,1,1,1,0]);%n*n
F = diag([0,0,0,0,0,0]);%n*n
%     [u_r,u_t,u_n]
R = diag([1,1,1]);%p*p

%预测区间
N = 5;
%MPC 控制器主函数
n = 6;
p = 3;

%初始状态变量值， n x 1 向量
X_K(:,1) =[a,e,i,Ohm,w,theta];
%定义输入矩阵 U_K， p x k 矩阵
U_K=zeros(p,k_steps);
%%
%模型线性化、离散化
[A,B,C,D] = linmod('satellite_GVE_model_sim');
% [A,B,C,D] = linmod('satellite_GVE_per_model_sim');

%离散化,零阶保持
[Ak,Bk,Ck,Dk] = c2dm(A,B,C,D,Ts,'zoh'); 

%Call MPC_Matrices 函数 求得 E,H矩阵 
[E,H]=MPC_Matrices(Ak,Bk,Q,R,F,N);
%%
%计算每一步的状态变量的值
for k = 1 : k_steps
%轨道根数更新
% a = X_K(1,k);
% e = X_K(2,k);
% i = X_K(3,k);
% w = X_K(4,k);
% Ohm = X_K(5,k);
% theta = X_K(6,k);

%其他更新
% p_s = a*(1-e^2);%轨道半通径
% h = sqrt(meu*p_s);%轨道角动量幅值
% b = a*sqrt(1-e^2);%半短轴
% r = a*(1-e^2)/(1+e*cos(theta));%地心距
% atm_dens_p = [2.488*(((-131.21+0.00299*r)+273.1)/216.6)^(-11.388)]/[0.2869*((-131.21+0.00299*r)+273.1)];

%求得U_K(:,k)
U_K(:,k) = Prediction(X_K(:,k),E,H,N,p);
% 卫星RTN坐标系下沿坐标轴的推力限制在1mN-4mN之间，换算为加速度约为3.7e-5~1.5e-4m/s^2
for i=1:3
    if U_K(i,k)>1.5e-4
        U_K(i,k) = 1.5e-4;
    elseif U_K(i,k)<-1.5e-4
        U_K(i,k) = -1.5e-4;
    elseif abs(U_K(i,k))<3.7e-5
        U_K(i,k) = 0;
    end
end

%计算第k+1步时状态变量的值
X_K(:,k+1)=(Ak*X_K(:,k)+Bk*U_K(:,k));
end
%% 绘制状态变量和输入的变化
subplot(4, 1, 1);
hold;
plot (X_K(1,:));
legend("da");
hold off;
title("Q = diag([1,1,1,1,1,0]); F = 0; R = diag([1,1,1])");
% title("无校正");

subplot(4, 1, 2);
hold;
for i =2 :size (X_K,1)-1
plot (X_K(i,:));
end
legend("de","di","dΩ","dw");
hold off;

subplot(4, 1, 3);
hold;
plot (X_K(size (X_K,1),:));
legend("dtheta");
hold off;

subplot (4, 1, 4);

hold;
for i =1 : size (U_K,1)
plot (U_K(i,:));
end
legend("u_r","u_t","u_n");
hold off;
%%
%功能函数
%MPC_Matrices.m
function[E , H]=MPC_Matrices(A,B,Q,R,F,N)

n=size(A,1);% A 是 n x n 矩阵, 得到 n
p=size(B,2);% B 是 n x p 矩阵, 得到 p

M=[eye(n);zeros(N*n,n)]; % 初始化 M 矩阵. M 矩阵是 (N+1)n x n的， 

% 它上面是 n x n 个 "I", 这一步先把下半部分写成 0 
C=zeros((N+1)*n,N*p); % 初始化 C 矩阵, 这一步令它有 (N+1)n x NP 个 0

% 定义M 和 C 
tmp=eye(n);%定义一个n x n 的 I 矩阵

%　更新Ｍ和C
for i=1:N % 循环，i 从 1到 N

rows =i*n+(1:n); %定义当前行数，从i x n开始，共n行 
C(rows,:)=[tmp*B,C(rows-n, 1:end-p)]; %将c矩阵填满
tmp= A*tmp; %每一次将tmp左乘一次A
M(rows,:)=tmp; %将M矩阵写满

end

% 定义Q_bar和R_bar
Q_bar = kron(eye(N),Q);

Q_bar = blkdiag(Q_bar,F);

R_bar = kron(eye(N),R);

% 计算G, E, H

G=M'*Q_bar*M; % G: n x n
E=C'*Q_bar*M; % E: NP x n
H=C'*Q_bar*C+R_bar; % NP x NP 

end

%Prediction
function u_k= Prediction(x_k,E,H,N,p)

U_k = zeros(N*p,1); % NP x 1
U_k = quadprog(H,E*x_k);
u_k = U_k(1:p,1); % 取第一个结果

end