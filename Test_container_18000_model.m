close all;
IsAxisTight = 1;
case_NO = 1;  
switch case_NO
    case 1
        % container 18000 TEU MMG 35-35
        clc;clear; IsAxisTight = 1;
        shipName = 'container_18000_MB_edit';
        testType = 'turncircle';% turncircle zigzag
        h = .51;t_rudderexecute = 2;t_final = 900;
        color_set = 'b-';rudder_colorSet = 'm--';
        manoeuver_angle = -35; % container_mb_blanke -35(右舵)
        dataOut_Tyukun_turncircle = case_list_turning_zigzag_steering_gear_container_18000(shipName,testType, manoeuver_angle,h,t_rudderexecute,t_final);
        plot_dataOut_Tyukun(dataOut_Tyukun_turncircle,color_set,rudder_colorSet,0,IsAxisTight);%figure(22);
        
    case 2
        clc;clear; IsAxisTight = 1;
        shipName = 'container_18000_MB_edit';
        testType = 'zigzag';% turncircle zigzag
        manoeuver_angle = [20,20];
        h = 0.5;t_rudderexecute = 1;t_final = [];
        color_set = 'r-.';rudder_colorSet = 'g--';
        dataOut_Tyukun_zigzag = case_list_turning_zigzag_steering_gear_container_18000(shipName,testType, manoeuver_angle,h,t_rudderexecute,t_final);
        plot_dataOut_Tyukun(dataOut_Tyukun_zigzag,color_set,rudder_colorSet,0,IsAxisTight);%figure(22);

end

