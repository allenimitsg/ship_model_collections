function [xdot,U] = container_18000_MB_edit(x,ui)
% [xdot,U] = kvlcc2(x,ui) 返回船舶合速度 U 以及状态变量向量: x = [ u v r x y psi delta  ]'
% kvlcc2 L = 320.0 m , nominal speed U0 = 7.97475 m/s = 15.5 knots. (1 kont = 0.5145 m/s)
%
% u     = (m/s)船中纵移速度相对于U0的扰动
% vm    = (m/s)船中横移速度
% r     = (rad/s)（弧度/秒）转首速率
% x     = position in x-direction (m)
% y     = position in y-direction (m)
% psi   = 首向角（rad）（弧度）
% delta = 实际舵角（rad）（弧度）
%
% 输入量:
% ui    = 命令舵角（rad）（弧度）

% This file is edited from kvlcc2_Yasukawa.m

% 检查输入量和状态变量
if (length(x)  ~= 10),error('x-vector must have dimension 10 !'); end
if (length(ui) ~= 2),error('ui must be 2 scalar input!'); end
%if nargin==2, U0 = 7.97475; end %如果输入个数为2，则自动赋值U0
% 外部环境
global Cr ct cq L B D dm H Tr Cb Cp xG xC Dp P AR KR HR Ae Z V n Cer beta beta_R alph_R K_tmp Cer
global Vw Rw Va Ra Af As Cax Cay Can windmark
global W_alfa W_h wavemark ttx tty Cxt Cyt Tsx Tsy ax ay ttx tty ut vt Xt Yt Nt Tsx Tsy Ct Js Va L1 B1 Cb1 dm1 dF rou L_BTpp  U
global Mx My Jz
rou = 1.025*1000; %海水密度
% 全尺度下的船舶参数，有量纲
LOA = 399.0;%(m) ??
Lpp = 385; % ??
L = Lpp;
B = 58.6;
dm = 19; %吃水 ??
dF = dm;dA = dm;
Tr = dA - dF; Tr = 0;
% there are two switch case in this code
H =100;
L_BTpp = 11.65*2;
Cb = 0.56;Cm = 0.99;Cp = Cb/Cm;
m = 251067*rou;   %(KG)
IzG = 0.011*((1/2)*rou*L^4*dm);% yang book P61 近似值 %Jz*((1/2)*rou*L^4*draft)/10; %(m)
Dp = 9.9;  %桨直径(m)
xG = 0;         %重心纵向坐标(m)
P  = 8.576;
AR = 75; %舵面积(m^2)
HR = 10.80; %舵高(m)
KR = 1.6;
Ae = 0.446;
Z  = 4;
n  = 76.5/60;
Cer=[-0.0001;2.0;0.3];
% V=23*1852/3600;
% 无量纲状态变量
delta_c = ui(1);   % delta_c = -ui such that positive delta_c -> positive r 输入舵角
u     = x(1);   %量纲
vm    = x(2);  %量纲
r     = x(3);  %量纲
psi   = x(6); %需要弧度
delta = x(9); %需要弧度
% U = sqrt(x(1)^2 + x(2)^2);
U = sqrt(u^2 + vm^2);
beta = atan2(-vm, u);
% 舵角限制
delta_max  = 35;           % max rudder angle      (deg)这里是角度，最大舵角
Ddelta_max =2.76;            % max rudder derivative (deg/s)这里是角度，最大转舵速率

% Rudder saturation and dynamics %changed from kvlcc2_Yasukawa to container_mb_blanke
if abs(delta_c) >= delta_max*pi/180
    delta_c = sign(delta_c)*delta_max*pi/180;
end

delta_dot = delta_c - delta;
if abs(delta_dot) >= Ddelta_max*pi/180
    delta_dot = sign(delta_dot)*Ddelta_max*pi/180;
end

Initial();
% 力和力矩模型
% [x1,yc]=currentforce(t,x,u);%流力
x_tmp = [delta,u,vm,r,psi,0,0];
[XP,NP]=screwforce([],x_tmp,u);%螺旋桨的作用力
YP = 0;
yr=rudderforce([],x_tmp,u);%舵力
% yw=windforce(t,x1,u);%风力
yh=hydrodynamic([],x_tmp,u);%水动力

% XH = yh(1)*0.5*rou*U^2*L*dm; YH = yh(2)*0.5*rou*U^2*L*dm; NH = yh(3)*0.5*rou*U^2*L^2*dm;
XH = yh(1)*0.5*rou*U^2*L*dm; YH = yh(2)*0.5*rou*U^2*L*dm; NH = yh(3)*0.5*rou*U^2*L^2*dm;
XR = yr(1); YR = yr(2); NR = yr(3);

X   = XH + XR + XP;
Y   = YH + YR + YP;
Nm  = NH + NR + NP;
%微分方程组
% xdot(1)=u_dot
% xdot(2)=vm_dot
% xdot(3)=r_dot
% xdot(4)=x_dot
% xdot(5)=y_dot
% xdot(6)=psi_dot
% xdot(7)=delta_dot
mx_have = Mx*(  (1/2)*rou*L^2*dm );
my_have = My*(  (1/2)*rou*L^2*dm );
Jz_have = Jz*(  (1/2)*rou*L^4*dm );
switch 1
    case 1
        u_dot=(X)/(m+mx_have);%纵向的加速度=水动力，推动力，风力，阻力，流力/船舶质量，附加质量
        vm_dot=(Y)/(m+my_have);
        r_dot=(Nm)/(IzG+Jz_have);%绕过O点的垂直轴的转矩，水动力、风力、阻力/惯性矩+附加惯性矩
        v = vm + xG*r; %重心横移速度
    case 2
        % 求解两个自由度的微分，符号求解
        tmp_a = (m + my_have);
        tmp_b = (m + mx_have);
        tmp_c = xG*m;
        tmp_d = (IzG + xG^2*m + Jz_have);
        tmp_e = u*r;
        % vm_dot = -(- e*c^2 + Nm*c - Y*d + b*d*e)/(- c^2 + a*d);
        % r_dot = (Nm*a - Y*c - a*c*e + b*c*e)/(- c^2 + a*d);
        A_linsolve = [tmp_a, tmp_c; tmp_c, tmp_d];
        B_linsolve = [Y-tmp_b*tmp_e;Nm-tmp_c*tmp_e];
        C_linsolve = linsolve(A_linsolve, B_linsolve);%C = A\B;
        vm_dot = C_linsolve(1);
        r_dot  = C_linsolve(2);
        u_dot=(X)/(m+mx_have);% [Fhull(1) Fp(1) Fw(1) Fr(1)]
        % vm_dot=(Y)/(m+my_have);% [Fhull(2) Fp(2) Fw(2) Fr(2)]
        % r_dot=(Nm)/(IzG+Jz_have);% [Fhull(3) Fp(3) Fw(3) Fr(3)]
        v = vm + xG*r; %重心横移速度
end
%delta_dot = 35 - delta;%舵角变化率
% 3-DOF运动方程  见（4）式
% Dimensional state derivatives  xdot = [ u v r x y psi p phi delta n ]'
X = X;
Y = Y;
N = Nm;
K = 0;
Xhyd = XH;
Yhyd = YH;
Nhyd = NH;
Khyd = 0;
Xrudder = XR;
Yrudder = YR;
Nrudder = NR;
Krudder = 0;
Xpropeller = XP;
Ypropeller = 0;
Npropeller = 0;
Kpropeller = 0;
alphaR = alph_R;

xdot =  [           u_dot
    vm_dot
    r_dot
    (cos(psi)*u-sin(psi)*v)                %这里的v要注意含义
    (sin(psi)*u+cos(psi)*v)
    r
    0
    0
    delta_dot
    0
    X
    Y
    N
    K
    Xhyd
    Yhyd
    Nhyd
    Khyd
    Xrudder
    Yrudder
    Nrudder
    Krudder
    Xpropeller
    Ypropeller
    Npropeller
    Kpropeller
    alphaR];

%解算过程


% hydrodynamic force caculation（水动力计算）
function yh=hydrodynamic(t,x,u)
global L Xvr Xvv Xrr Xvvvv Yv Yr Yvv Yrr Yvvr Yvrr Nv Nr Nrr Nvv Nvvr Nvrr U
resi=resistant(t,x,u);
x_old = x; x(2) = x(2)/U; x(3) = x(3)/U; x(4) = x(4)/(U/L);
yh(1)=resi+Xvr*x(3)*x(4)+Xvv*x(3)^2+Xrr*x(4)^2+Xvvvv*x(3)^4;
yh(2)=(Yv*x(3)+Yr*x(4))+Yvv*x(3)*abs(x(3))+...
      +Yrr*x(4)*abs(x(4))+(Yvvr*x(3)^2*x(4)+Yvrr*x(3)*x(4)^2);
yh(3)=(Nv*x(3)+Nr*x(4))+Nrr*x(4)*abs(x(4))+Nvv*x(3)*abs(x(3))+...
      +(Nvrr*x(3)*x(4)^2+Nvvr*x(3)^2*x(4));
x = x_old;
% end hydrodynamic

% resistant calculation（阻力计算）
function resi=resistant(t,x,u)
global xh Cr L B dm tr U
global pp_Holtrop_Mennen_82_KVLCC2; % better than pp_Holtrop_Mennen_82_container_180000
switch 2
    case 1
        fn=log10(x(2)*L/(1.188*1e-6));
        cf=0.4631/fn^2.6;
        fn=x(2)/sqrt(9.8*L);
        Cr=[2.659067;-26.710230;90.341408;-94.853523];% resistant
        cr=Cr(1)*fn^3+Cr(2)*fn^4+Cr(3)*fn^5+Cr(4)*fn^6;
        resi=xh*(cr/4+cf)*x(2)^2/6;% ??
    case 2
        u = x(2);
        Rn=log10(u*L/(1.188*1e-6));
        cf=0.075/(Rn-2)^2;
        cr = ppval(pp_Holtrop_Mennen_82_KVLCC2, u);
        dcr=(B/dm-2.4)*0.005;
        cr=cr*(1+dcr);
        cr=(cr+cf);
        resi=xh*cr*x(2)/U*abs(x(2))/U*(1+0.143*tr)/6;% ??
end
% end resistant

function Initial()
global L B D dm Tr Cb xG xC Dp P AR KR tr
global xh Iz M Mx My Jz Xvr Xvv Xrr Yv Yr Yrr Yvv Yvvr Yvrr 
global Nv Nr Nvv Nrr Nvvr Nvrr Xvvvv
global tR rR lR aH xH xR wp0 wr0 CR dh tp0 ra Kt Cp  Cer
global Af As Wax Way Wan Wavx Wavy Wavn
global W_alfa W_h wk we w Wn wavemark 
% xh=(Cb*L*B*dm)^(2/3)*(3.432+0.305*L/B+0.443*B/dm-0.643*Cb);
xh=(1.54*dm+0.45*B+0.904*B*Cb+0.026*Cb*B/dm)*L;
xh=-xh/(L*dm);
M=2*Cb/(L/B);
Iz=(0.055*Cb+0.029)*M;
Mx=0.01*(0.398+11.97*Cb*(1+3.73*dm/B)-2.89*Cb*L/B*(1+1.13*dm/B)+0.175*Cb*(L/B)*(L/B)*(1+0.541*dm/B)-1.107*L/B*dm/B);
My=0.882-0.54*Cb*(1-1.6*dm/B)-0.156*(1-0.673*Cb)*L/B+...
    +0.826*(dm/B)*(L/B)*(1-0.678*dm/B)-0.638*Cb*(dm/B)*(L/B)*(1-0.669*dm/B);
k=0.01*(33-76.85*Cb*(1-0.784*Cb)+3.43*L/B*(1-0.63*Cb));
Jz=M*k*k;

tr=Tr/dm; 
Xvr=((1.11*Cb-1.07)+(1.11*Cb-0.07)*0.208*tr)*My;
Xvv=0.4*B/L-0.006*L/dm;
Xrr=Cer(1)*L/dm;
Xvvvv=4*B/L-0.002*L/dm;

k=2*dm/L;
Yv=-(0.5*pi*k+1.4*Cb*B/L)*(1+(25*Cb*B/L-2.25)*tr);
Yr=((M+Mx)-1.5*Cb*B/L)*(1+(571*(dm*(1-Cb)/B)^2-81*dm*(1-Cb)/B+2.1)*tr);
Nv=-k*(1-tr);
Nr=-(0.54*k-k^2)*(1+(34*Cb*B/L-3.4)*tr);

Yvv=(-2.5*(1-Cb)*B/(dm-0.5))*(1-(35.7*Cb*B/L-2.5)*tr);
Yrr=(0.343*Cb*dm/B-0.07)*(1+(45*Cb*B/L-8.1)*tr);
Yvrr=(-5.95*(1-Cb)*dm/B)*(1+(40*(1-Cb)*dm/B-2)*tr);
Yvvr=(1.5*Cb*dm/B-0.65)*(1+(110*(1-Cb)*dm/B-9.7)*tr);

Nvv=(0.96*(1-Cb)*dm/B-0.066)*(1+(58*(1-Cb)*dm/B-5)*tr);
Nrr=(0.5*Cb*B/L-0.09)*(1-(30*Cb*B/L-2.6)*tr);
Nvvr=(-57.5*(Cb*B/L)^2+18.4*(Cb*B/L)-1.6)*(1+(3*Cb*B/L-1)*tr);
Nvrr=(0.5*Cb*B/L-0.05)*(1+(48*(Cb*B/L)^2-16*(Cb*B/L)+1.3)*100*tr);


tR=0.2618+0.0539*Cb-0.1755*Cb^2;
rR=Cer(2)*(-22.2*(Cb*B/L)^2+0.02*(Cb*B/L)+0.68);
lR=-1;
aH=0.6784-1.3374*Cb+1.8992*Cb^2;
xH=-0.45;
xR=-0.5;
% end Initial

% screw force calculation（螺杆内力计算）
function [Xp,Np]=screwforce(t,x,u)
global beta beta_P L B dm Cp Cb Dp beta_R dF n rou L_BTpp wP_s wP_p U
% P ---- 桨力
r = x(4);
d = dm; u = x(2);
xP = -0.5; %螺旋桨坐标找不到,此处的桨坐标从贾书找到
beta_P = beta - xP*r/(U/L); %这里用无量纲算有量纲
if beta > 0
    kp_s = 5.4/1000; kp_p = -2.2/1000;
else
    kp_p = 5.4/1000; kp_s = -2.2/1000;
end
%wP = wP0*exp(-4*beta_P^2);
k0 = 0.34;
        k1 = -0.19;
        k2 = -0.1553;
A_BT = 3*3; hB = 3/2; k = 0.5; % wangzhanwang Thesis guess it may be size as container height and width ?? k
Rn=log10(x(2)*L/(1.188*1e-6));
cf=0.075/(Rn-2)^2; % from Ship_Project_A_Report1_FINAL.pdf
nP = n;
if(isempty(beta_R))
    beta_R = 0;
end
tp0 = 0.50*Cp-0.18; % ?? Cp ship prismatic coefficient from wangzhengwang
lcb = xP*L/L*100; % ??
gamaA = B/d*(1.3*(1-Cb)-3.1*lcb);
kt_tmp = 0.00023*gamaA*L/Dp-0.028;
f = kt_tmp*beta_R;
tp = tp0-f;
if(dF/L<= 0.04)
	C4 = dF/L;
else
	C4 = 0.04;
end
C3 = 0.56*A_BT^1.5/(B*dm*(0.31*sqrt(A_BT)+dF-hB));
C2 = exp(-1.89*sqrt(C3));
C_A = 0.006*(L+100)^(-0.16) - 0.00205 + 0.003*sqrt(L/7.5)*Cb^4*C2*(0.04-C4);
Cv = (1+k)*cf+ C_A;
wP0 = 0.3095*Cb + 10*Cv*Cb - 0.23*Dp/sqrt(B*dm);

wP_p = 1 - ((1-wP0) + kp_p*beta_P); %wP
Jp = u*(1-wP_p)/(nP*Dp);
KT = k2*Jp^2 + k1*Jp + k0;
Tp = rou*nP^2*Dp^4*KT; %

wP_s = 1 - ((1-wP0) + kp_s*beta_P); %wP
Jp = u*(1-wP_s)/(nP*Dp);
KT = k2*Jp^2 + k1*Jp + k0;
Ts = rou*nP^2*Dp^4*KT; % 

Xp = (1-tp)*(Tp+Ts);  %对结果有影响需要调试
Np = (1-tp)*L_BTpp*(Tp-Ts);

%end screwforce

% rudder force calculation（舵力计算）
function yr=rudderforce(t,x,u)
global L P Dp HR AR KR tR rR lR aH xH xR wp0 wr0 CR dh ra br U Cb 
global beta beta_P wP_s wP_p dm n L_BTpp rou alph_R K_tmp
U=sqrt(x(2)^2+x(3)^2); u = x(2);
CR=AR/(L*L*dm)*6.13*KR/(2.25+KR);
tR = 1 - (0.7382-0.0539*Cb+0.1755*Cb^2);

% beta = beta_P;
if beta >= 0
    wr_p = 1-(1.1+6.1*(beta)/1000); wr_s = 1-(1.1+2.5*(beta)/1000000);
    wp_p = 0.97-(5.4*(beta)/1000);  wp_s = 0.97+(2.2*(beta)/1000000);%
    gammaR_p = 0.7+0.36*beta/10^8;  gammaR_s = 0.5;
else
    wr_s = 1-(1.1+6.1*(beta)/1000); wr_p = 1-(1.1+2.5*(beta)/1000);
    wp_s = 0.97-(5.4*(beta)/1000);  wp_p = 0.97+(2.2*(beta)/1000);
    gammaR_s = 0.7+0.36*beta/10^8;  gammaR_p = 0.5;
end

s=1-u*(1-wp_p)/(P*n);
if x(1)>=0 k2=1.065;
else k2=0.935; end
if (1-s)<0.3 ah=aH*(1-s)/0.3;
else ah=aH; end;ah = aH;
K =0.6*(1-wp_p)/(1-wr_p);
Eta = Dp/HR; 
gg=Eta*K*(2-(2-K)*s)*s/(1-s)^2;
ur=(1-wr_p)*U*sqrt(1+k2*gg);
alph_R = x(1)-gammaR_p*beta/(1-wr_p);
f_alph = 1.743*alph_R+7.201*abs(alph_R)*(alph_R)-11.77*alph_R^3;
FN_p = -0.5*rou*AR*ur^2*f_alph;

s=1-u*(1-wp_s)/(P*n);
if x(1)>=0 k2=1.065;
else k2=0.935; end
if (1-s)<0.3 ah=aH*(1-s)/0.3;
else ah=aH; end
K =0.6*(1-wp_s)/(1-wr_s);% ?? K = 0.016363636363636; 
Eta = Dp/HR; 
gg=Eta*K*(2-(2-K)*s)*s/(1-s)^2;
ur=(1-wr_s)*U*sqrt(1+k2*gg);
alph_R = x(1)-gammaR_s*beta/(1-wr_s);
f_alph = 1.743*alph_R+7.201*abs(alph_R)*(alph_R)-11.77*alph_R^3;
FN_s = -0.5*rou*AR*ur^2*f_alph; % FN_p = FN_s;
%s*pi/90+

yr(1)=(1-tR)*(FN_s*sin(x(1))+FN_p*sin(x(1)));
yr(2)=(1+ah)*(FN_s*cos(x(1))+FN_p*cos(x(1)));
yr(3)=(xR+ah*xH)*L*(FN_s*cos(x(1))+FN_p*cos(x(1)))+(1-tR)*L_BTpp*(FN_p*sin(x(1))-FN_s*sin(x(1)));
% end rudderforce

function yw=windforce(t,x,u)
global Va Ra Cax Cay Can Wax Way Wan windmark
ua=x(2)+Va*cos(x(5)-Ra);
va=x(3)-Va*sin(x(5)-Ra);
Ua=ua*ua+va*va;
if windmark==0
    for i=1:3 yw(i)=0; end
else
    if ua==0 ra=0;
    else ra=atan2(-va,ua); end
    Bp=abs(ra*18/pi);
    bint=fix(Bp);
    Bp=Bp-bint;
    if bint>=19
        cx=Cax(bint+1); cy=Cay(bint+1); cn=Can(bint+1);
    else
        cx=Cax(bint+1)+Bp*(Cax(bint+2)-Cax(bint+1));
        cy=Cay(bint+1)+Bp*(Cay(bint+2)-Cay(bint+1));
        cn=Can(bint+1)+Bp*(Can(bint+2)-Can(bint+1));
    end
    if ra<0.0
        cy=-cy;
        cn=-cn;
    end
    yw(1)=Wax*Ua*cx;
    yw(2)=Way*Ua*cy;
    yw(3)=Wan*Ua*cn;
end

function [x1,yc]=currentforce(t,x,u)
global M Mx My Vw Rw
x1=x;
x1(2)=x(2)+Vw*cos(x(5)-Rw);
x1(3)=x(3)-Vw*sin(x(5)-Rw);
yc(1)=Vw*x(4)*sin(x(5)-Rw)*(M+Mx);
yc(2)=Vw*x(4)*cos(x(5)-Rw)*(M+My);
% end currentforce
