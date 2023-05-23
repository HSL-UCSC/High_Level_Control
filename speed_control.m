%% Clean
clc;
close all;

%% Robot Dog Network Parameters
% this IP is the vm ip
Robot_Dog_IP = '192.168.123.161';
Robot_Dog_Port = 1145;

%% Robot Dog Command Initialized
Control_Command = zeros(1,11,'single');
%velocity walking
Control_Command(1)=2;
if isempty(gcp('nocreate'))
    parpool;
end
futureResult = parallel.FevalFuture;
%% Feedback Control Parameters

% Porportional constant on velocity action
K_P_x = 1;
K_P_z = 1;
K_P_yaw = 0.5;

% Integral
K_I_x = 0.15;
K_I_z = 0.15;
K_I_yaw = 0.03;

% Derivative
K_D_x = 0.08;
K_D_z = 0.08;
K_D_yaw = 0.05;

% limit
p_x_limit = 0;       % m/s
p_z_limit = 0;        % m/s
p_yaw_limit = 40*pi/180; % rad/s

i_x_limit = 0;
i_z_limit = 0;
i_yaw_limit = 5*pi/180;

d_x_limit = 0;
d_z_limit = 0;
d_yaw_limit = 5*pi/180;
%% Control Setting
% MODE
% Mode 1: Target point.
% Mode 2: Dog will follow Way_Points.
% Mode 3: Dog will follow the path
Control_Mode=1;
Control_Speed=0.5;
Stop_Distance=0.5*Control_Speed^2;
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

%% figure for movtion track
fig = figure();
ax = axes('Parent',fig);

arrow_length=0.2;
%circle for draw
circle_center =[0,0];
circle_radius =1.65;
circle_theta = linspace(0,2*pi,100);
circle_x=circle_center(1)+circle_radius*cos(circle_theta);
circle_y=circle_center(2)+circle_radius*sin(circle_theta);
%% Robot dog command
%     Control_Command()
%
%     +(11) +(9)  -(11)
%             |
%     +(10)  dog  -(10)
%             |
%           -(9)
%
%% Motive coordiate frame
% wall wall wall wall wall
%        ^ z
%        |
%        |
% x <----O y(pointing up)
%
%
% wall computer wall

%% Init Parameters
% index for way points
Way_Point_index=1;

integral_x = 0;
integral_z = 0;
integral_yaw = 0;

previous_error_x = 0;
previous_error_z = 0;
previous_error_yaw = 0;


Dog_Pos_Record=[];
Dog_Pos_Record_Index = 1;
i=0;
%% Main Loop
while true
    % get position from camera
    % async_robot_dog(Robot_Dog_IP,Robot_Dog_Port,Control_Command);
    [Dog_Pos] = Get_Dog_Postion(theClient, Dog_ID); %[time, x, z, yaw]
    if i==0
        i=1;
        Dog_Pos_Record=[Dog_Pos_Record ; Dog_Pos];
    else
        if ~isequal(Dog_Pos_Record(end,:),Dog_Pos) %if not the same values
            i=i+1;
            Dog_Pos_Record=[Dog_Pos_Record ; Dog_Pos];
            Rotation_matrix = [cosd(Dog_Pos(4)), -sind(Dog_Pos(4)) ; sind(Dog_Pos(4)),cosd(Dog_Pos(4)) ];

            d_dog_pos = Dog_Pos_Record(i,:)-Dog_Pos_Record(i-1,:); %[dtime, dx, dz, dyaw]
            real_time = Dog_Pos_Record(i-1,1)-Dog_Pos_Record(1,1);
            Real_Dog_Speed_Room = [d_dog_pos(2)/d_dog_pos(1), d_dog_pos(3)/d_dog_pos(1)];
            Real_Dog_Speed_Dog = Rotation_matrix*Real_Dog_Speed_Room';
            %Dog_Speed_Record=[Dog_Speed_Record;real_time,Real_Dog_Speed_R]; %[rtime, z_speed, x_speed]

            if Control_Mode==1
                Vector_D_T = Target_Point-[Dog_Pos(2) Dog_Pos(3)]; % Get vector
                Norm_VDT = norm(Vector_D_T);      % Calculate Distance
                scale=Control_Speed/Norm_VDT;
                Ref_Speed_Room=scale*Vector_D_T;
                Ref_Speed_Dog = Rotation_matrix*Ref_Speed_Room';
                Error_Speed_Dog = Ref_Speed_Dog-Real_Dog_Speed_Dog;
                %error yaw calculate
                Error_Yaw=Yaw_Controllor(yaw_set,Dog_Pos(4),Vector_D_T);
                %PID Control
                Control_x=PID_Controllor(K_P_x,K_I_x,K_D_x,d_dog_pos(1),Error_Speed_Dog(1),integral_x,previous_error_x,p_x_limit,i_x_limit,d_x_limit);
                Control_z=PID_Controllor(K_P_z,K_I_z,K_D_z,d_dog_pos(1),Error_Speed_Dog(2),integral_z,previous_error_z,p_z_limit,i_z_limit,d_z_limit);
                Control_yaw=PID_Controllor(K_P_yaw,K_I_yaw,K_D_yaw,d_dog_pos(1),Error_Yaw,integral_yaw,previous_error_yaw,p_yaw_limit,i_yaw_limit,d_yaw_limit);
                if Norm_VDT> Stop_Distance
                    Control_Command(10) = Control_x;   %x
                    Control_Command(9)  = Control_z;   %z
                    Control_Command(11) = Control_yaw; %yaw
                else
                    Control_Command = zeros(1,11,'single');
                    %velocity walking
                    Control_Command(1)=2;
                    %break;
                end

            end

        end
    end
    async_robot_dog(Robot_Dog_IP,Robot_Dog_Port,Control_Command);
end

