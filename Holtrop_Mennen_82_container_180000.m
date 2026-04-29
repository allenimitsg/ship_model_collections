function pp = Holtrop_Mennen_82_container_180000()

%%%% Open water resistance %%%%
%%%% %%%%
%%%% Holtrop-Mennen 82 %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reference: Ship_Project_A_Report1_FINAL.pdf
%% Input
Lpp = 360;
L = Lpp;
B = 58.6;
T = 17;
v = 0.1:0.5144:23*0.5144;
nu = 1.187 * 10^(-6);
rho = 1025;
Cp = 251067/(0.9980*B*T*L);% ¨Œ/(Cm*B*d*L) % Prismatic coefficient
Cb = 0.56; % Block coefficient
Cm = 0.9980; % Cm = Am/(B*d) % Mid-ship coefficient
Cwp = 0.8855; % £¿£¿£¿ Cwp = Aw/(L*B) % Water plane coefficient
Abt = 14; % £¿£¿£¿ % Transversal area of bulb
hb = 3; % £¿£¿£¿ % Height of center of transversal crossection of bulb at waterline
At = 0; % Immersed transom area at 0 speed
g = 9.81;
Vdisp = 312622; % Displacement volume
lcb = 200; % £¿£¿£¿ % Longitudinal center of buoyancy
S = (1.54*T+0.45*B+0.904*B*Cb+0.026*Cb*B/T)*Lpp; % Wetted surface area for Twin p Twin R Ship
Sapp = 0; % £¿£¿£¿ % Surface area of the Azipod-unit outside of the hull
%% Resistance components
Fn = v./sqrt(g*L); % Froude number
c12 = (T/L)^0.2228446;
c13 = 1.003;
LR = L * (1-Cp + 0.06 * Cp * lcb / (4 * Cp - 1));
k1 = c13 * (0.93 + c12 * (B/LR)^0.92497 * (0.95 - Cp)^(-0.521448) * ...
(1 - Cp + 0.0225 * lcb)^0.6906);
% frictional resistance (ITTC-57)
Rn = v .* L / nu; % Reynolds number
C.F = 0.075 ./ (log10(Rn)-2).^2;
R.F = 1/2 .* rho .* v.^2 .* S .* C.F;
R.F = R.F.' .* k1; % Frictional resistance
% appendage resistance due to the Azipod-unit
k2 = 1.5;
R.APP = 0.5 .* rho .* v.^2 .* Sapp .* k2 .* C.F;
R.APP = R.APP.';
% water resistance
d = -0.9;
iE = 35; % Waterline entrance angle
c3 = 0.56 * Abt^1.5 / (B*T * (0.31 * sqrt(Abt) + T - hb));
c7 = B/L;
c1 = 2223105 * c7.^3.78613 * (T/B)^1.07961 * (90 - iE)^(-1.37565);
c2 = exp(-1.89 * sqrt(c3));
c5 = 1 - 0.8 * At ./ (B*T*Cm);
lambda = 1.446 * Cp - 0.36;
c16 = 8.07981 * Cp - 13.8673 * Cp^2 + 6.984388 * Cp^3;
m1 = 0.0140407 * L/T - 1.75254 * Vdisp.^(1/3)/L - 4.79323 * B/L - c16;
c15 = -1.69385;
m2 = c15 .* Cp.^2 .* exp(-0.1 .* Fn.^(-2));
R.W = c1 .* c2 .* c5 .* Vdisp .* rho .* g .* ...
exp(m1 .* Fn.^d + m2 .* cos(lambda .* Fn.^(-2)));
R.W = R.W.'; % Water resistance
% bulb resistance
Pb = 0.56 .* sqrt(Abt) ./ (T - 1.5 .* hb);
Fni = v ./ sqrt(g .* (T - hb - 0.25 * sqrt(Abt)) + 0.15 * v.^2);
R.B = 0.11 .* exp(-3.*Pb.^(-2)) .* Fni.^3 .* Abt.^1.5 .* rho .* g ./ (1 + Fni.^2);
R.B = R.B.';
% model ship correlation resistance
c4 = 0.04;
Ca = 0.006 * (L + 100)^(-0.16) - 0.00205 + 0.003 * ...
sqrt(L/7.5) * Cb^4 * c2 * (0.04 - c4);
R.A = 1/2 * rho * v.^2 * S * Ca;
R.A = R.A.';
%% Total resistance
R.F(isnan(R.F)) = 0;
R.APP(isnan(R.APP)) = 0;
R.W(isnan(R.W)) = 0;
R.B(isnan(R.B)) = 0;
R.A(isnan(R.A)) = 0;

R_total = R.F + R.APP + R.W + R.B + R.A;
% figure(1)
% area([R.F, R.APP, R.W, R.B , R.A], 'Tag', 'January Data');
% legend('R.F', 'R.APP', 'R.W', 'R.B ', 'R.A');title('Resistance total');
% figure(2)
% area(v, [R.F, R.APP, R.W, R.B , R.A]/(1/2*rho*L*T)./v'./v', 'Tag', 'January Data');
% legend('R.F', 'R.APP', 'R.W', 'R.B ', 'R.A');title('Resistance total pie');

% figure(3)
pp = spline(v, R_total/(1/2*rho*L*T)./v'./v');
% yy = ppval(pp, 0:0.1:max(v));
% plot(0:0.1:max(v),yy, '*');title('Resistance total pie');
end
