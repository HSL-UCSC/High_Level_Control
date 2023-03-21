%% Clean
clc; 
close all;

%% Check List:
% 1.Power on Robot Dog
% 2.Connect Robot Dog Wifi: Unitree_Go393319A
% 3.Open Virtual Machine
% 4.Running /Desktop/unitree_matlab/build/udp_link press Enter to begin
% 5.Open Motive Check Broadcast and Network Interface
% 6.You are good for running 

%% Robot Dog Network Parameters
% this IP is the vm ip
Robot_Dog_IP = '192.168.12.184';
Robot_Dog_Port = 1145;

%% Robot Dog Command Initialized
Control_Command = zeros(1,11,'single');

%% Feedback Control Parameters
K_P = 0.5; % Porportional constant on velocity action
K_I = 1;  % Integral
K_D = 1;
% constant on velocity action
K_Y = pi/180; % Proportional constant on angle action

%% Control Setting 
% Mode 1: Dog will keep face to yaw_set and move to target point.
% Mode 2: Dog will turn to target point and walk to it.

Control_Mode=1;
Target_Point=[0 0];
yaw_set = -1; 

% [0,360)
% -1: Disable yaw control

% wall wall wall wall wall
%           0
%           ^ z
%           |
%           |
% 90 x <----O      270
%
%          180
%
% wall computer wall

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
[Dog_Pos] = Get_Dog_Postion(theClient, Dog_ID); %[time, z, x, yaw]
fig = figure();
ax = axes('Parent',fig);
line_handle = plot(ax,Dog_Pos(3),Dog_Pos(2),'o');

xlabel('X')
ylabel('Z')

arrow_length=0.2;

%velocity walking
Control_Command(1)=2; 
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
% wall computer wall
%% Loop 
while true
    % get position from camera
    [Dog_Pos] = Get_Dog_Postion(theClient, Dog_ID); %[time, z, x, yaw]

    %% feedback control
    if Control_Mode == 1
        if yaw_set == -1
            error_yaw = yaw_set-Dog_Pos(4);
            error_yaw_command=0;

        elseif yaw_set >=0 && yaw_set<360
            error_yaw = yaw_set-Dog_Pos(4);
            if abs(error_yaw)>180
                error_yaw_command=(360+error_yaw);
            else
                error_yaw_command=error_yaw;
            end
        end

        error_Z = Target_Point(1)-Dog_Pos(2);
        error_X = -Target_Point(2)-Dog_Pos(3);

        error_Z_rotated = error_Z*cosd(error_yaw)-error_X*sind(error_yaw);
        error_X_rotated = error_Z*sind(error_yaw)+error_X*cosd(error_yaw);

        Control_Command(11) = error_yaw_command*K_Y;
        Control_Command(9) = error_Z_rotated*K_P;
        Control_Command(10) = error_X_rotated*K_P;



    end
    disp(Control_Command);
    Robot_Dog(Robot_Dog_IP,Robot_Dog_Port,Control_Command);

    %50Hz
    pause(0.02);
    %% draw 
    dy=arrow_length*cosd(Dog_Pos(4));
    dx=arrow_length*sind(Dog_Pos(4));
    quiver(Dog_Pos(3),Dog_Pos(2),dx,dy,'r','LineWidth',0.2,'MaxHeadSize',2);
    set(gca,'XDir','reverse');
    xlim(ax,[-2,2]);
    ylim(ax,[-2,2]);
    %set(line_handle, 'XData',Dog_Pos(3),'YData',Dog_Pos(2))
    drawnow;
end



