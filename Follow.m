%% Clean
clc;
clear;
close all;

%% Robot Dog Network Parameters
% this IP is the vm ip
Robot_Dog_IP = '192.168.254.132';
Robot_Dog_Port = 1145;

% Robot Dog Command Initialized
Control_Command = zeros(1,11,'single');
%velocity walking
Control_Command(1)=2;
% if isempty(gcp('nocreate'))
%     parpool;
% end
% futureResult =
% parallel.FevalFuture;
%% Feedback Control Parameters

% Porportional constant on velocity action
K_P_x = 0.6;
K_P_z = 0.6;
K_P_yaw = 0.3;

% Integral
K_I_x = 1.8;
K_I_z = 1.8;
K_I_yaw = 0.05;

% Derivative
K_D_x = 0.02;
K_D_z = 0.02;
K_D_yaw =0.005;

% limit
p_x_limit = 0;       % m/s
p_z_limit = 0;        % m/s
p_yaw_limit = 20*pi/180; % rad/s

i_x_limit = 1;
i_z_limit = 1;
i_yaw_limit = 40*pi/180;

d_x_limit = 0.2;
d_z_limit = 0.2;
d_yaw_limit = 2*pi/180;

%% Control Setting

Control_Speed=0.6;

Switch_Distance=0.8;

% Target Point
%[x,z]
Target_Point=[0 0];

% YAW
% [0,360)
% -1: Disable yaw control
% -2: Control Yaw to the motion direction

% Yaw
% wall wall wall wall wall
%          0,359.9..
%           ^ z
%           |
%           |
% 90 x <----O      270
%
%          180
%
% wall computer wall
yaw_set = -2;


%% Instantiate client object to run Motive API commands

% Check list:
% 1.Broadcast Frame Date
% 2.Network Interface: Local Loopback

% https://optitrack.com/software/natnet-sdk/
% Create Motive client object
dllPath = fullfile('d:','StDroneControl','NatNetSDK','lib','x64','NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath); % Add API function calls
theClient = NatNetML.NatNetClientML(0);

% Create connection to localhost, data is now being streamed through client object
HostIP = '127.0.0.1';
theClient.Initialize(HostIP, HostIP);

Dog_ID = 1; % Rigid body ID of the drone from Motive
Car_ID = 2;
% Robot dog command
%     Control_Command()
%
%     +(11) +(9)  -(11)
%             |
%     +(10)  dog  -(10)
%             |
%           -(9)
%

% Motive coordiate frame
% wall wall wall wall wall
%        ^ z
%        |
%        |
% x <----O y(pointing up)
%
%
% wall computer wall


%% Init Parameters

integral_x = 0;
integral_z = 0;
integral_yaw = 0;

previous_error_x = 0;
previous_error_z = 0;
previous_error_yaw = 0;

Dog_Speed_Record=[];
Dog_Pos_Record=[];
Car_R=[]
while true
    [init_time, x, z, yaw] = Get_Dog_Postion(theClient, Dog_ID); %[time, x, z, yaw];
    Dog_Pos_Record=[0, x, z, yaw];
    i=1;
    if init_time ~= 0
        break
    end
end

%% Main Loop
while true
    % get position from camera
    % async_robot_dog(Robot_Dog_IP,Robot_Dog_Port,Control_Command);
    [time, x, z, yaw] = Get_Dog_Postion(theClient, Dog_ID); %[time, x, z, yaw]
    [Car_time, Car_x, Car_z, Car_yaw] = Get_Dog_Postion(theClient, Car_ID);
    if abs(Car_x)<2.5 || abs(Car_z)<1.6
        Target_Point = [Car_x,Car_z];
    end
    real_time = time-init_time;
    if ~isequal(Dog_Pos_Record(end,:), [real_time, x, z, yaw]) %if not the same values
        i=i+1;
        Dog_Pos_Record=[Dog_Pos_Record ; real_time, x, z, yaw];
        Rotation_matrix = [cosd(yaw), -sind(yaw) ; sind(yaw),cosd(yaw) ];
        Car_R=[Car_R;Target_Point,real_time];
        d_dog_pos = Dog_Pos_Record(i,:)-Dog_Pos_Record(i-1,:); %[dtime, dx, dz, dyaw]
        
        
        Real_Dog_Speed_Room = [d_dog_pos(2)/d_dog_pos(1), d_dog_pos(3)/d_dog_pos(1)];
        Real_Dog_Speed_Dog = Rotation_matrix*Real_Dog_Speed_Room';
        
        Vector_D_T = Target_Point-[x z]; % Get vector
        
        Norm_VDT = norm(Vector_D_T);      % Calculate Distance
        scale=Control_Speed/Norm_VDT;
        Ref_Speed_Room=scale*Vector_D_T;
        Ref_Speed_Dog = Rotation_matrix*Ref_Speed_Room';
        Error_Speed_Dog = Ref_Speed_Dog-Real_Dog_Speed_Dog;
        
        %error yaw calculate
        [Error_Yaw,Mode2_Yaw_Ref]=Yaw_Controllor(yaw_set,yaw,Vector_D_T);
        %PID Control
        [Control_x,integral_x]=PID_Controllor(K_P_x,K_I_x,K_D_x,d_dog_pos(1),Error_Speed_Dog(1),integral_x,previous_error_x,p_x_limit,i_x_limit,d_x_limit);
        [Control_z,integral_z]=PID_Controllor(K_P_z,K_I_z,K_D_z,d_dog_pos(1),Error_Speed_Dog(2),integral_z,previous_error_z,p_z_limit,i_z_limit,d_z_limit);
        [Control_yaw,integral_yaw]=PID_Controllor(K_P_yaw,K_I_yaw,K_D_yaw,d_dog_pos(1),Error_Yaw,integral_yaw,previous_error_yaw,p_yaw_limit,i_yaw_limit,d_yaw_limit);
        
        previous_error_x = Error_Speed_Dog(1);
        previous_error_z = Error_Speed_Dog(2);
        previous_error_yaw = Error_Yaw;
        disp([Real_Dog_Speed_Dog',Error_Speed_Dog',Control_x,Control_z,Control_yaw])
        
        Dog_Speed_Record=[Dog_Speed_Record;real_time,Real_Dog_Speed_Dog',Ref_Speed_Dog',Error_Speed_Dog',Control_x,Control_z,yaw,Mode2_Yaw_Ref,Control_yaw];
        %[rtime, real_x_speed, real_z_speed, ref_x_speed, ref_z_speed, error_x, error_z]
        
        %async_robot_dog(Robot_Dog_IP,Robot_Dog_Port,Control_Command);
        if Norm_VDT> Switch_Distance
            Control_Command(10) = Control_x;   %x
            Control_Command(9)  = Control_z;   %z
            Control_Command(11) = Control_yaw; %yaw
        else
            Control_Command(10) = 0;   %x
            Control_Command(9)  = 0;   %z
            Control_Command(11) = Control_yaw; %yaw
            %             if Error_Yaw < 5
            %                 Control_Command(11) = 0;
            %             end
            integral_z=0;
            integral_x=0;
            integral_yaw=0;
            previous_error_yaw=0;
            previous_error_z=0;
            previous_error_x=0;
        end
        
        
        Robot_Dog(Robot_Dog_IP,Robot_Dog_Port,Control_Command);
    end
end
%% Speed figures
figure;
plot(Dog_Pos_Record(:,2),Dog_Pos_Record(:,3),'DisplayName','Dog');
title('Robot Dog follow Car')
xlabel('X');
ylabel('Z');
set(gca,'XDir','reverse');
hold on;
rectangle('Position',[-1.5,-1.5,3,3],'Curvature',[1,1]);
plot(Car_R(:,1),Car_R(:,2),'DisplayName','Car');
xlim([-4,4]);
ylim([-2,2]);
daspect([1 1 1]);
legend;
hold off;
