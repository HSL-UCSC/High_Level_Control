%% Clean
clc;
clear;
close all;

%% Robot Dog Network Parameters
% this IP is the vm ip
Robot_Dog_IP = '192.168.123.161';
Robot_Dog_Port = 1145;

% Robot Dog Command Initialized
Control_Command = zeros(1,11,'single');
%velocity walking
Control_Command(1)=2;
% if isempty(gcp('nocreate'))
%     parpool;
% end
% futureResult = parallel.FevalFuture;
%% Feedback Control Parameters

% Porportional constant on velocity action
K_P_x = 0.6;
K_P_z = 0.6;
K_P_yaw = 0.07;

% Integral
K_I_x = 2;
K_I_z = 2;
K_I_yaw = 0;

% Derivative
K_D_x = 0.02;
K_D_z = 0.02;
K_D_yaw = 0;

% limit
p_x_limit = 0;       % m/s
p_z_limit = 0;        % m/s
p_yaw_limit = 40*pi/180; % rad/s

i_x_limit = 1;
i_z_limit = 1;
i_yaw_limit = 5*pi/180;

d_x_limit = 0.2;
d_z_limit = 0.2;
d_yaw_limit = 5*pi/180;

%% Control Setting
% MODE
% Mode 1: Target point.
% Mode 2: Dog will follow Way_Points.
% Mode 3: Dog will follow the path
Control_Mode=1;
Control_Speed=0.6;
Stop_Distance=0.25*Control_Speed^2;
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
yaw_set = 0;

% THRESHOLD
% Distance Threshold to switch to next way point
Distance_Threshold = 0.15;

% Cricel way points 18
Way_Points_center =[0,0];
Way_Points_radius =1.65;
Way_Points_theta = linspace(0,2*pi,50);
Way_Points_x=Way_Points_center(1)+Way_Points_radius*cos(Way_Points_theta);
Way_Points_z=Way_Points_center(2)+Way_Points_radius*sin(Way_Points_theta);

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
    real_time = time-init_time;
    if ~isequal(Dog_Pos_Record(end,:), [real_time, x, z, yaw]) %if not the same values
        i=i+1;
        Dog_Pos_Record=[Dog_Pos_Record ; real_time, x, z, yaw];
        Rotation_matrix = [cosd(yaw), -sind(yaw) ; sind(yaw),cosd(yaw) ];

        d_dog_pos = Dog_Pos_Record(i,:)-Dog_Pos_Record(i-1,:); %[dtime, dx, dz, dyaw]


        Real_Dog_Speed_Room = [d_dog_pos(2)/d_dog_pos(1), d_dog_pos(3)/d_dog_pos(1)];
        Real_Dog_Speed_Dog = Rotation_matrix*Real_Dog_Speed_Room';

        Vector_D_T = Target_Point-[x z]; % Get vector
        Norm_VDT = norm(Vector_D_T);      % Calculate Distance
        scale=Control_Speed/Norm_VDT;
        Ref_Speed_Room=scale*Vector_D_T;
        Ref_Speed_Dog = Rotation_matrix*Ref_Speed_Room';

        Error_Speed_Dog = Ref_Speed_Dog-Real_Dog_Speed_Dog;

        
        %[rtime, real_x_speed, real_z_speed, ref_x_speed, ref_z_speed, error_x, error_z]

        %error yaw calculate
        Error_Yaw=Yaw_Controllor(yaw_set,yaw,Vector_D_T);
        %PID Control
        [Control_x,integral_x]=PID_Controllor(K_P_x,K_I_x,K_D_x,d_dog_pos(1),Error_Speed_Dog(1),integral_x,previous_error_x,p_x_limit,i_x_limit,d_x_limit);
        [Control_z,integral_z]=PID_Controllor(K_P_z,K_I_z,K_D_z,d_dog_pos(1),Error_Speed_Dog(2),integral_z,previous_error_z,p_z_limit,i_z_limit,d_z_limit);
        [Control_yaw,integral_yaw]=PID_Controllor(K_P_yaw,K_I_yaw,K_D_yaw,d_dog_pos(1),Error_Yaw,integral_yaw,previous_error_yaw,p_yaw_limit,i_yaw_limit,d_yaw_limit);

        previous_error_x = Error_Speed_Dog(1);
        previous_error_z = Error_Speed_Dog(2);
        previous_error_yaw = Error_Yaw;
        disp([Real_Dog_Speed_Dog',Error_Speed_Dog',Control_x,Control_z,Control_yaw])

        Dog_Speed_Record=[Dog_Speed_Record;real_time,Real_Dog_Speed_Dog',Ref_Speed_Dog',Error_Speed_Dog',Control_x,Control_z]; 

        if Norm_VDT> 0.2
            Control_Command(10) = Control_x;   %x
            Control_Command(9)  = Control_z;   %z
            Control_Command(11) = Control_yaw; %yaw
        else
            Control_Command = zeros(1,11,'single');
            %velocity walking
            Control_Command(1)=2;
            %async_robot_dog(Robot_Dog_IP,Robot_Dog_Port,Control_Command);
            Robot_Dog(Robot_Dog_IP,Robot_Dog_Port,Control_Command);
            break;
        end
        %async_robot_dog(Robot_Dog_IP,Robot_Dog_Port,Control_Command);
        Robot_Dog(Robot_Dog_IP,Robot_Dog_Port,Control_Command);
    end
end

figure;
plot(Dog_Speed_Record(:,1),Dog_Speed_Record(:,2),'DisplayName','Measure Speed');
title('Robot Dog X Speed')
hold on;
plot(Dog_Speed_Record(:,1),Dog_Speed_Record(:,4),'LineWidth',1,'DisplayName','Ref Speed');

plot(Dog_Speed_Record(:,1),Dog_Speed_Record(:,6),'LineWidth',1,'DisplayName','Error');

plot(Dog_Speed_Record(:,1),Dog_Speed_Record(:,8),'LineWidth',1,'DisplayName','Control');
legend;
xlabel('Time(t)');
ylabel('Speed(m/s)');
hold off;
