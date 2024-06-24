3%%
%清屏
clear all;
clc;
%%
%各种常量

% 推进器1
% ENPULSION的MICRO R3可提供的最高比冲为1000s,适用于50kg级别的微卫星
% 动态推力范围200uN~1.35mN
% 额定推力1mN
% 特定一次脉冲1500~6000s
% 质量3.9kg
% 总脉冲量>=50000Ns
% 额定推进下的功率90-100W
% 热保持功率10-15W
% 推进器2
% spaceware microXL
% 湿质量115kg
% 总推进量52000Ns
% 功率100W-150W
% 推力4mN~7mN

%使用轨道参数为Astrid微卫星LEO轨道,来自www.heavens-above.com
% 轨道偏心率:	0.0039866
% 轨道倾角:	     82.9236°
% 近地点高度:	960 km
% 远地点高度:	1019 km
% 升交点赤经:	272.9481°
% 近地点幅角:	297.2910°
% 每日绕地圈数:	13.72814543 %卫星理论周期 T = 2*pi()*sqrt(a^3/meu) = 104.8940s
% 初始时刻的平近点角:	62.4188°
% 初始时刻的轨道圈数:	45809
% 功率，来自维基百科
% 11.88 瓦（有效载荷），38.5 瓦（标称）
G = 6.67e-11;%重力常数，m^3/(kg·s^2)
m_e = 5.965e24;%地球质量，kg
meu = G*m_e;

% T = -131.21+0.00299*960000;近地点大气温度，来自NASA大气模型
% p = 2.488*[(T+273.1)/216.6]^(-11.388);%近地点大气压强
% atm_dens_p = p/[0.2869*(T+273.1)];%近地点大气密度kg/m^3
atm_dens_p = 2.7544e-16;%近地点大气密度kg/m^3

% syms h;
% r(h) = [2.488*[(-131.21+0.00299*h+273.1)/216.6]^(-11.388)]/[0.2869*(-131.21+0.00299*h+273.1)];
% beta = -(dr/dh/r);
beta = 1.2296e-05;%大气标高的倒数m^-1

n_s = 13.72814543*2*pi()/24/60/60;%真近点角角速率均值,rads/s
R_e = 6378.2e3;%地球赤道半径km

J2 = 1082.63e-6;%地球引力场二阶带谐系数
s = 0.2*0.2;%受力面积m^2,8U
Cd = 2.2;%气动系数,一般取2.2
m_s = 27;%卫星质量(算上预计的推进系统)
Kd = s*Cd/2/m_s;%大气阻力系数
%%
%变量初始化

%误差
da = 500;
de = -0.0001;
di = 0.005/180*pi();
dOhm = 0;
dw = 0;
dM_s = 0;

%非被控变量
f = 0;%rad,真近点角初始值
r = 960000;%m,地心距初始值
%%
%状态空间方程（不更新的部分）
C = eye(6,6);
D = zeros(6,3);
%%
%MPC控制器的初始化

%采样时间
Ts = 600;%s

%step数量
k_steps=500;%3.5日
% k_steps=1008;%一周

%二次规划系数矩阵
%    [da,de,di,dOhm,dw,dM_s]
Q = diag([1,20,60,10,1,0]);%n*n
F = diag([0,0,0,0,0,0]);%n*n
%     [u_r,u_t,u_n]
R = diag([1,1,1]);%p*p

%预测区间
N = 5;
%MPC 控制器主函数
n = 6;
p = 3;

%初始状态变量值， n x 1 向量
X_K(:,1) =[da,de,di,dOhm,dw,dM_s];
%定义输入矩阵 U_K， p x k 矩阵
U_K=zeros(p,k_steps);
%%
%计算每一步的状态变量的值
for k = 1 : k_steps
%轨道根数更新
a = (960e3+1019e3)/2+R_e-X_K(1,k);%m
e = 0.0039866-X_K(2,k);
i = 82.9236/180*pi()-X_K(3,k);%82.9236°
w = 297.2910/180*pi()-X_K(4,k);%近地点辐角297.2910°
Ohm = 272.9481/180*pi()-X_K(5,k);%升交点赤经272.9481°
M_s = 62.4188/180*pi()-X_K(6,k);%平均近点角62.4188°

%其他更新
p_s = a*(1-e^2);%轨道半通径
h = sqrt(meu*p_s);%轨道角动量幅值
b = a*sqrt(1-e^2);%半短轴
f = f + k*Ts*h/r^2;%真近点角
r = a*(1-e^2)/(1+e*cos(f));%地心距
theta = w+f;%纬度辐角
I0 = besseli(0,beta*a*e);%第一类修正贝塞尔函数
I1 = besseli(1,beta*a*e);
I2 = besseli(2,beta*a*e);
I3 = besseli(3,beta*a*e);

%状态空间表达式更新
f1 = [I0+2*e*I1+(3*e^2/4)*(I0+I2)]*exp(-beta*a*e);
f2 = [I1+(e/2)*(I0+I2)-(e^2/8)*(5*I1+I3)]*exp(-beta*a*e);
f1_da = beta*e*[-I0+I1+2*e*(I0-I1/beta/a/e-I1)+3*e^2*(2*I1-I0-I2-2*I2/beta/a/e)/4]*exp(-beta*a*e);
f1_de = a*f1_da/e+[2*I1+3*e*(I0+I2)/2]*exp(-beta*a*e);
f2_da = 2*beta*e*[I0-I1-I1/beta/a/e+e*(2*I1-2*I2/beta/a/e-I0-I2)/2-e^2*(5*I0-5*I1/beta/a/e+I2-3*I3/beta/a/e-5*I1-I3)/8]*exp(-beta*a*e);
f2_de = a*f2_da/e+[I0+I2-e*(5*I1+I3)/2]*exp(-beta*a*e);

S11 = -sqrt(meu/a)*Kd*atm_dens_p*(f1+2*a*f1_da);
S12 = -2*sqrt(meu*a)*Kd*atm_dens_p*f1_de;
S21 = sqrt(meu/a^3)*Kd*atm_dens_p*(f2-2*a*f2_da)/2;
S22 = -sqrt(meu/a)*Kd*atm_dens_p*f2_de;
S41 = 21*J2*(R_e/p_s)^2*n_s*cos(i)/4/a;
S42 = -6*J2*R_e^2*n_s*a*e*cos(i)/(p_s^3);
S43 = 3*J2*n_s*(R_e/p_s)^2*sin(i)/2;
S51 = -S41*(5*cos(i)^2-1)/2/cos(i);
S52 = -S42*(5*cos(i)^2-1)/2/cos(i);
S53 = -15*J2*n_s*(R_e/p_s)^2*sin(i)*cos(i)/2;
S61 = -3*n_s*[1+7*J2*sqrt(1-e^2)*(R_e/p_s)^2*(3*cos(i)^2-1)/4]/2/a;
S62 = 9*J2*n_s*(R_e/p_s)^2*(3*cos(i)^2-1)*e/4/[(1-e^2)^2.5];
S63 = -9*J2*n_s*(R_e/p_s)^2*sqrt(1-e^2)*sin(i)*cos(i)/2;
S = [S11 S12 0 0 0 0;
     S21 S22 0 0 0 0;
     0   0   0 0 0 0;
     S41 S42 S43 0 0 0;
     S51 S52 S53 0 0 0;
     S61 S62 S63 0 0 0];

B11 = 2*a^2*e*sin(f)/h;
B12 = 2*a^2*p_s/r/h;
B21 = p_s*sin(f)/h;%控制e
B22 = [(p_s+r)*cos(f)+r*e]/h;%控制e
B33 = r*cos(theta)/h;
B43 = r*sin(theta)/h/sin(i);
B51 = -p_s*cos(f)/h/e;
B52 = (p_s+r)*sin(f)/h/e;
B53 = -r*sin(theta)*cos(i)/h/sin(i);
B61 = b*(p_s*cos(f)-2*r*e)/a/h/e;
B62 = -b*(p_s+r)*sin(f)/a/h/e;
B = [B11 B12 0;
     B21 B22 0;
     0   0  B33;
     0   0  B43;
     B51 B52 B53;
     B61 B62 0];

%离散化,前向差分
% Sk = eye(6,6)+Ts*S;
% Bk = Ts*B;
% Ck = C;
% Dk = D;

%离散化,零阶保持
[Sk,Bk,Ck,Dk] = c2dm(S,B,C,D,Ts,'zoh'); 

%Call MPC_Matrices 函数 求得 E,H矩阵 
[E,H]=MPC_Matrices(Sk,Bk,Q,R,F,N);

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
X_K(:,k+1)=(Sk*X_K(:,k)+Bk*U_K(:,k));
end
%%
% 容忍的控制范围
% 在1000km轨道上，按正弦函数计算，100m对应5.73e-3deg，20m对应1.15e-3deg,10m对应5.73e-4deg，5m对应2.86e-4deg
% 距离上选取10m作为极限
% i 选取1e-3deg≈1.7453e-05rads作为极限
% e 选取5e-05作为极限
% w 暂且0.0005
% 重点控制a e i三项，对M不做控制
%% 绘制状态变量和输入的变化

subplot(4, 1, 1);
hold;
plot (X_K(1,:));
legend("da");
hold off;
title("Q = diag([1,20,60,10,1,0]); F = 0; R = diag([1,1,1])");
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
legend("dM");
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