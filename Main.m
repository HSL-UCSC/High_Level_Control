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
%velocity walking
Control_Command(1)=2; 
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
Target_Point=[0.3 -0.2]; %[x,z]
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
fig = figure();
ax = axes('Parent',fig);

arrow_length=0.2;

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

    %% Feedback control

    % Calculate Distance
    Point_Dog = [Dog_Pos(2) Dog_Pos(3)];  %[x,z]
    Vector_PD_TP = Target_Point-Point_Dog;
    Norm_Vector = norm(Vector_PD_TP);
    Rotation_matrix = [cosd(Dog_Pos(4)), -sind(Dog_Pos(4)) ; sind(Dog_Pos(4)),cosd(Dog_Pos(4)) ];
    %% Enable Control
    if Norm_Vector > 0.1 %Distance > 0.1M 
        %% Mode 1
        if Control_Mode == 1
            if yaw_set == -1
                error_yaw_command=0;

            elseif yaw_set >=0 && yaw_set<360
                error_yaw = yaw_set-Dog_Pos(4);
                if abs(error_yaw)>180
                    error_yaw_command=(360+error_yaw);
                else
                    error_yaw_command=error_yaw;
                end
            end
            Vector_rotated = Rotation_matrix*Vector_PD_TP';
           
            Control_Command(11) = error_yaw_command*K_Y;
            Control_Command(10) = Vector_rotated(1)*K_P; %x
            Control_Command(9) = Vector_rotated(2)*K_P;  %z
        end
%         %% Mode2
%         if Control_Mode == 2
%             Point_Dog = [Dog_Pos(2) Dog_Pos(3)];
%             Vector_PD_TP = Target_Point-Point_Dog;
%             Norm_Vector = norm(Vector_PD_TP);
%             if(Norm_Vector>0.5)
%                 angle_r = atan2(-Vector_PD_TP(1),Vector_PD_TP(2));
%                 yaw_set = rad2deg(angle_r);
%                 if yaw_set >= -90
%                     yaw_set = yaw_set+90;
%                 else
%                     yaw_set = 360+(yaw_set+90);
%                 end
%             end
% 
%             error_yaw = yaw_set-Dog_Pos(4);
%             if abs(error_yaw)>180
%                 error_yaw_command=(360+error_yaw);
%             else
%                 error_yaw_command=error_yaw;
%             end
%             error_Z = Target_Point(1)-Dog_Pos(2);
% 
%             error_Z_rotated = error_Z*cosd(Dog_Pos(4))-error_X*sind(Dog_Pos(4));
% 
% 
%             Control_Command(11) = error_yaw_command*K_Y;
%             Control_Command(10) = error_Z_rotated*K_P;
%             Control_Command(10) = 0;
% 
%         end
    else
        Control_Command(11) = 0;
        Control_Command(9) = 0;
        Control_Command(10) = 0;
    end
    disp(Control_Command);
    Robot_Dog(Robot_Dog_IP,Robot_Dog_Port,Control_Command);


    %% draw 

    circle_center =[0,0];
    circle_radius =1.65;
    circle_theta = linspace(0,2*pi,100);
    circle_x=circle_center(1)+circle_radius*cos(circle_theta);
    circle_y=circle_center(2)+circle_radius*sin(circle_theta);
    plot(ax,circle_x,circle_y,'b-');
    xlabel('X')
    ylabel('Z')
    hold on;
    plot(ax,Target_Point(1),Target_Point(2),'x');
    ax.DataAspectRatio=[1 1 1];
    dy=arrow_length*cosd(Dog_Pos(4));
    dx=arrow_length*sind(Dog_Pos(4));
    quiver(Dog_Pos(2),Dog_Pos(3),dx,dy,'r','LineWidth',0.2,'MaxHeadSize',2);
    set(gca,'XDir','reverse');
    xlim(ax,[-3,3]);
    ylim(ax,[-2,2]);
    hold off;
    drawnow;
    %% 50Hz
    pause(0.02);
end



