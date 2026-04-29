function [t,u,v,r,x,y,psi,U,dataOut_Tyukun] = zigzag_mb_edit(ship,x,ui,t_final,t_rudderexecute,h,maneuver)
% ZIGZAG      [t,u,v,r,x,y,psi,U] = zigzag(ship,x,ui,t_final,t_rudderexecute,h,maneuver)
%             performs the zig-zag maneuver, see ExZigZag.m
%
% Inputs :
% 'ship'          = ship model. Compatible with the models under .../gnc/VesselModels/
% x               = initial state vector for ship model
% ui              = [delta,:] where delta=0 and the other values are non-zero if any
% t_final         = final simulation time
% t_rudderexecute = time control input is activated
% h               = sampling time
% maneuver        = [rudder angle, heading angle]. Default 20-20 deg that is: maneuver = [20, 20] 
%                    rudder is changed to maneuver(1) when heading angle is larger than maneuver(2)
%
% Outputs :
% t               = time vector
% u,v,r,x,y,psi,U = time series
%
% Author:    Thor I. Fossen
% Date:      22th July 2001
% Revisions: 15th July 2002, switching logic has been modified to handle arbitrarily maneuvers
% ________________________________________________________________
%
% MSS GNC is a Matlab toolbox for guidance, navigation and control.
% The toolbox is part of the Marine Systems Simulator (MSS).
%
% Copyright (C) 2008 Thor I. Fossen and Tristan Perez
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
% 
% E-mail: contact@marinecontrol.org
% URL:    <http://www.marinecontrol.org>

if nargin>7 | nargin<6, error('number of inputs must be 6 or 7'); end
if t_final<t_rudderexecute, error('t_final must be larger than t_rudderexecute'); end


N = round(t_final/h);               % number of samples
xout = zeros(N+1,9);                % memory allocation
data_output_T_yukun_inside = zeros(N+1,41); % mb edit data output Tyukun

disp('Simulating...')

u_ship=ui;

for i=1:N+1,
    time = (i-1)*h;
    
    psi = x(6)*180/pi;
    r   = x(3);
    
    if round(time)==t_rudderexecute,  
        u_ship(1)=maneuver(1)*pi/180; 
    end
    
    if round(time) > t_rudderexecute, 
        if (psi>=maneuver(2) & r>0),
            u_ship(1) = -maneuver(1)*pi/180; 
        elseif (psi<=-maneuver(2) & r<0),
            u_ship(1) = maneuver(1)*pi/180; 
        end   
    end
 
    [xdot,U] = feval(ship,x,u_ship);       % ship model
        
    % % mb add 
    if(~isempty(strfind(ship,'NORudderDelay')))% no rudder delay ship manoeuvring model
        data_output_T_yukun_inside(i,11) = u_ship(1);
    else
        data_output_T_yukun_inside(i,11) = x(9); % delta % mb edit for yukun type data output
    end
    data_output_T_yukun_inside(i,12) = x(10);% n
    data_output_T_yukun_inside(i,19) = xdot(1);% udot
    data_output_T_yukun_inside(i,20) = xdot(2);% vdot
    data_output_T_yukun_inside(i,21) = xdot(3);% rdot
    data_output_T_yukun_inside(i,22) = x(8);% Phi
    data_output_T_yukun_inside(i,23) = xdot(8); % Phidot
    data_output_T_yukun_inside(i,41) = xdot(7); % Phidotdot
    
    if(length(xdot) > 10) % condition for including force and moment 
        data_output_T_yukun_inside(i,24) = xdot(11);%X
        data_output_T_yukun_inside(i,25) = xdot(12);%Y
        data_output_T_yukun_inside(i,26) = xdot(13);%N
        data_output_T_yukun_inside(i,27) = xdot(14);%K
        data_output_T_yukun_inside(i,28) = xdot(15);%Xhyd
        data_output_T_yukun_inside(i,29) = xdot(16);%Yhyd
        data_output_T_yukun_inside(i,30) = xdot(17);%Nhyd
        data_output_T_yukun_inside(i,31) = xdot(18);%Khyd
        data_output_T_yukun_inside(i,32) = xdot(19);%Xrudder
        data_output_T_yukun_inside(i,33) = xdot(20);%Yrudder
        data_output_T_yukun_inside(i,34) = xdot(21);%Nrudder
        data_output_T_yukun_inside(i,35) = xdot(22);%Krudder
        data_output_T_yukun_inside(i,36) = xdot(23);%Xpropeller
        data_output_T_yukun_inside(i,37) = xdot(24);%Ypropeller
        data_output_T_yukun_inside(i,38) = xdot(25);%Npropeller
        data_output_T_yukun_inside(i,39) = xdot(26);%Kpropeller
        if(27 == length(xdot))
            data_output_T_yukun_inside(i,40) = xdot(27);%AlphaR
        end
        xdot = xdot(1:length(x));
    end
    
    xout(i,:) = [time,x(1:6)',U,u_ship(1)];  
    
    x = euler2(xdot,x,h);                     % Euler integration
    
end

% time-series
t     = xout(:,1);
u     = xout(:,2); 
v     = xout(:,3);         
r     = xout(:,4)*180/pi; 
x     = xout(:,5);
y     = xout(:,6);
psi   = xout(:,7)*180/pi;
U     = xout(:,8);
delta_c = xout(:,9)*180/pi;
 

% 育鲲数据格式
% %1秒Second,2时分秒HHMMSS,3纬度Lat,4经度Lon,5航向Psi,6纵向速度对地u
% %,7纵向速度对水u,8横向速度对地v,9横向速度对水v,10转向速度r,11舵角Delta
% %,12车速n,13风向Uwr,14风速Uwr,15螺距P,16 heading angle,17 V,18 beta(rad)
% %,19纵向加速度对地 udot,20横向加速度对地 vdot,'21转艏加速度 rdot'
% %,'22横倾角 fai Phi','23横倾角加速度 faidot', , 24 Xtotal, 25 Ytotal
% %,26 Ntotal, 27 Ktotal, 28 Xhyd, 29 Yhyd, 30 Nhyd, 31 Khyd, 32 Xrudder
% %, 33 Yrudder, 34 Nrudder, 35 Krudder, 36 Xpropeller, 37 Ypropeller, 38 Npropeller, 39 Kpropeller, 40 AlphaR, 41 Phidotdot
dataOut_Tyukun = [t,zeros(N+1,1),x,y,deg2rad(psi),u,...
    zeros(N+1,1),v,zeros(N+1,1),deg2rad(r),data_output_T_yukun_inside(:,11),...
    data_output_T_yukun_inside(:,12),zeros(N+1,1),zeros(N+1,1),zeros(N+1,1),zeros(N+1,1),U,atan2(-v,u),...
    data_output_T_yukun_inside(:,19),data_output_T_yukun_inside(:,20),data_output_T_yukun_inside(:,21),...
    data_output_T_yukun_inside(:,22),data_output_T_yukun_inside(:,23), data_output_T_yukun_inside(:,24),...
    data_output_T_yukun_inside(:,25), data_output_T_yukun_inside(:,26), data_output_T_yukun_inside(:,27),...
    data_output_T_yukun_inside(:,28), data_output_T_yukun_inside(:,29), data_output_T_yukun_inside(:,30),...
    data_output_T_yukun_inside(:,31), data_output_T_yukun_inside(:,32), data_output_T_yukun_inside(:,33),...
    data_output_T_yukun_inside(:,34), data_output_T_yukun_inside(:,35), data_output_T_yukun_inside(:,36),...
    data_output_T_yukun_inside(:,37), data_output_T_yukun_inside(:,38), data_output_T_yukun_inside(:,39),...
    data_output_T_yukun_inside(:,40),data_output_T_yukun_inside(:,41)];


% % plots
% figure(1)
% plot(x,y,'linewidth',2),grid,axis('equal'),xlabel('x-position'),ylabel('y-position')
% title('Zig-zag test')
% figure(2)
% subplot(211),plot(t,psi,'linewidth',2)
% hold on
% plot(t,delta_c,'r')
% hold off
% xlabel('time (s)'),title('yaw angle \psi (deg)'),grid
% legend('\psi','\delta_c')
% subplot(212),plot(t,U),xlabel('time (s)'),title('speed U (m/s)'),grid
