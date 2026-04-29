% clc;clear;
function dataOut_Tyukun = case_list_turning_zigzag_steering_gear_container_18000(shipName,testType,manoeuver_angle,h,t_rudderexecute,t_final)
% path_free = genpath('master_project-master');
% addpath(path_free);
% path_free = genpath('examples_mb_edit');
% addpath(path_free);
% path_free = genpath('gnc_mfiles_mb_edit');
% addpath(path_free);
% path_free = genpath('vessel_models_mb_edit');
%addpath(path_free);
% path_free = genpath('plot_mb');
% addpath(path_free);
% path = genpath('E:\111\学生\朱金善\学生\船舶阻力与KVLCC2\Holtrop_Mennen_82\');
% addpath(path);
global pp_Holtrop_Mennen_82_container_180000;
pp_Holtrop_Mennen_82_container_180000 = Holtrop_Mennen_82_container_180000();
global pp_Holtrop_Mennen_82_KVLCC2; % turn circle figure better than pp_Holtrop_Mennen_82_container_180000
% path = genpath('E:\文档\实验室\zhu\船舶阻力与KVLCC2\Holtrop_Mennen_82');
% addpath(path);
pp_Holtrop_Mennen_82_KVLCC2 = Holtrop_Mennen_82_KVLCC2();

delta_c = 20*pi/180; % rudder angle for maneuver (rad)
% h = 0.1; % sampling time (sec)
% shipName = 'container' ;
% testType = 'zigzag';%turncircle zigzag


switch testType
    case 'zigzag'
        %%
        if(strcmp('container_18000_MB_edit',shipName))
            if(isempty(t_final)), t_final =  750;  end         % final simulation time (sec)
            x  = [23*1852/3600;0;0;0;0;0;0;0;0;76.5/60];   % x  = [ u v r x y psi delta ]' (initial values)
            delta_c = 0*pi/180;                    % delta_c = 0 for time t < t_rudderexecute
            n_c     = x(end);                          % n_c = propeller revolution in rpm

            ui = [delta_c, n_c];
            [t,u,v,r,x,y,psi,U,dataOut_Tyukun] = zigzag_mb_edit(shipName,x,ui,t_final,t_rudderexecute,h,manoeuver_angle);

        end

    case 'turncircle'
        %%
        if(strcmp('container_18000_MB_edit',shipName))
            if(isempty(t_final)), t_final =  750;   end         % final simulation time (sec)
            x  = [23*1852/3600;0;0;0;0;0;0;0;0;76.5/60];   % x  = [ u v r x y psi delta ]' (initial values)
            delta_c = -manoeuver_angle*pi/180; % delta_c = 0 for time t < t_rudderexecute
            n_c     = x(end); % n_c = propeller revolution in rpm

            ui = [delta_c, n_c];

            [t,u,v,r,x,y,psi,U,dataOut_Tyukun] = turncircle_mb_edit(shipName,x,ui,t_final,t_rudderexecute,h);
        end
end
