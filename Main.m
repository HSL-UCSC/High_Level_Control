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
dt = 0.05; % time

% Porportional constant on velocity action
K_P_x = 0.65;
K_P_z = 0.65;
K_P_yaw = pi/180;

% Integral
K_I_x = 0.01;
K_I_z = 0.01;
K_I_yaw = 0;

% Derivative
K_D_x = 0.1;
K_D_z = 0.1;
K_D_yaw = 0;

% limit
propostional_x_limit = 0.6;
propostional_z_limit = 0.6;
propostional_yaw_limit = 0.6;

integral_x_limit = 0.1;
integral_z_limit = 0.1;
integral_yaw_limit = 1*pi/180;

derivative_x_limit = 0.3;
derivative_z_limit = 0.3;
derivative_yaw_limit = 1*pi/180;
%% Control Setting
% MODE
% Mode 1: Dog will go to Target_point.
% Mode 2: Dog will follow Way_Points.
Control_Mode=2;

% Target Point
%[x,z]
Target_Point=[0 0];

% YAW
% [0,360)
% -1: Disable yaw control

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
Distance_Threshold = 0.30;

% Cricel way points 18
Way_Points_center =[0,0];
Way_Points_radius =1.65;
Way_Points_theta = linspace(0,2*pi,50);
Way_Points_x=Way_Points_center(1)+Way_Points_radius*cos(Way_Points_theta);
Way_Points_z=Way_Points_center(2)+Way_Points_radius*sin(Way_Points_theta);
%% Path Planning 
% generate random points
random_points_number = 4;
random_points_r = 1.6;
random_points_center_x = 0;
random_points_center_z = 0;
random_points = zeros(random_points_number,2);

for i = 1:random_points_number
    angle = 2*pi*rand;
    random_r = random_points_r * sqrt(rand);
    random_point_x = random_points_center_x + random_r * cos(angle);
    random_point_z = random_points_center_z + random_r * sin(angle);
    random_points(i,:)=[random_point_x random_point_z];
end

figure;
viscircles([random_points_center_x, random_points_center_z], random_points_r,'LineStyle','--','Color','k');
hold on;
scatter(random_points(:,1), random_points(:,2),'red','filled');
axis equal;
hold off;



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

breakflag = 0;
Dog_Pos_Record=[];
Dog_Pos_Record_Index = 1;
[Dog_Pos] = Get_Dog_Postion(theClient, Dog_ID);
time = Dog_Pos(1);

%% Main Loop
while true
    % get position from camera
    [Dog_Pos] = Get_Dog_Postion(theClient, Dog_ID); %[time, z, x, yaw]
    Dog_Pos_Record=[Dog_Pos_Record;Dog_Pos Dog_Pos(1)-time];
    Dog_Pos_Record_Index = Dog_Pos_Record_Index +1;
    if Control_Mode == 2
        Target_Point = [Way_Points_x(Way_Point_index) Way_Points_z(Way_Point_index)];
    end


    %% Feedback control

    % Calculate Distance
    Point_Dog = [Dog_Pos(2) Dog_Pos(3)];  %[x,z]
    Vector_PD_TP = Target_Point-Point_Dog; % Get vector
    Norm_Vector = norm(Vector_PD_TP);      % Calculate norm

    % rotation matrix to turn vector to robot dog frame
    Rotation_matrix = [cosd(Dog_Pos(4)), -sind(Dog_Pos(4)) ; sind(Dog_Pos(4)),cosd(Dog_Pos(4)) ];

    if Norm_Vector > Distance_Threshold
        %% Mode 1
        % not control yaw
        if yaw_set == -1
            error_yaw_command=0;

            % control yaw
        elseif yaw_set >=0 && yaw_set<360
            error_yaw = yaw_set-Dog_Pos(4); %get error
            if abs(error_yaw)>180
                error_yaw_command=(360+error_yaw);
            else
                error_yaw_command=error_yaw;
            end
        end
        %vector in robot dog frame = error_x error_z
        Vector_rotated = Rotation_matrix*Vector_PD_TP';

        %% propostional
        propostional_x   = Vector_rotated(1) * K_P_x;
        propostional_z   = Vector_rotated(2) * K_P_z;
        propostional_yaw = error_yaw_command * K_P_yaw;

        % propostional limit
        if propostional_x > propostional_x_limit
            propostional_x = propostional_x_limit;
        elseif propostional_x < -propostional_x_limit
            propostional_x = -propostional_x_limit;
        end

        if propostional_z > propostional_z_limit
            propostional_z = propostional_z_limit;
        elseif propostional_z < -propostional_z_limit
            propostional_z = -propostional_z_limit;
        end

        if propostional_yaw > propostional_yaw_limit
            propostional_yaw = propostional_yaw_limit;
        elseif propostional_yaw < -propostional_yaw_limit
            propostional_yaw = -propostional_yaw_limit;
        end

        %% integral
        integral_x   = integral_x   + K_I_x   * Vector_rotated(1) * dt;
        integral_z   = integral_z   + K_I_z   * Vector_rotated(2) * dt;
        integral_yaw = integral_yaw + K_I_yaw * error_yaw_command * dt;

        % integral limit
        if integral_x > integral_x_limit
            integral_x = integral_x_limit;
        elseif integral_x < -integral_x_limit
            integral_x = -integral_x_limit;
        end

        if integral_z > integral_z_limit
            integral_z = integral_z_limit;
        elseif integral_z < -integral_z_limit
            integral_z = -integral_z_limit;
        end

        if integral_yaw > integral_yaw_limit
            integral_yaw = integral_yaw_limit;
        elseif integral_yaw < -integral_yaw_limit
            integral_yaw = -integral_yaw_limit;
        end


        %% derivative
        derivative_x   = K_D_x   * (Vector_rotated(1) - previous_error_x)  /dt;
        derivative_z   = K_D_z   * (Vector_rotated(2) - previous_error_z)  /dt;
        derivative_yaw = K_D_yaw * (error_yaw_command - previous_error_yaw)/dt;

        previous_error_x = Vector_rotated(1);
        previous_error_z = Vector_rotated(2);
        previous_error_yaw = error_yaw_command;

        % derivative limit
        if derivative_x > derivative_x_limit
            derivative_x = derivative_x_limit;
        elseif derivative_x < -derivative_x_limit
            derivative_x = -derivative_x_limit;
        end

        if derivative_z > derivative_z_limit
            derivative_z = derivative_z_limit;
        elseif derivative_z < -derivative_z_limit
            derivative_z = -derivative_z_limit;
        end

        if derivative_yaw > derivative_yaw_limit
            derivative_yaw = derivative_yaw_limit;
        elseif derivative_yaw < -derivative_yaw_limit
            derivative_yaw = -derivative_yaw_limit;
        end


        %% set command
        Control_Command(10) = propostional_x   + integral_x   + derivative_x;   %x
        Control_Command(9)  = propostional_z   + integral_z   + derivative_z;   %z
        Control_Command(11) = propostional_yaw + integral_yaw + derivative_yaw; %yaw

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
        % finished, stop running

        % clean integral
        integral_x = 0;
        integral_z = 0;
        integral_yaw = 0;

        if Control_Mode == 1
            Control_Command(11) = 0;
            Control_Command(10) = 0; %x
            Control_Command(9) = 0;  %z
            breakflag = 1;
        elseif Control_Mode == 2
            % go to next way point
            if length(Way_Points_z)>Way_Point_index
                Way_Point_index=Way_Point_index+1;
            else
                %finished stop running
                Control_Command(11) = 0;
                Control_Command(10) = 0; %x
                Control_Command(9) = 0;  %z
                breakflag = 1;
            end
        end
    end
    % print command
    disp(Control_Command);
    % send command to vitrual machine
    Robot_Dog(Robot_Dog_IP,Robot_Dog_Port,Control_Command);

    %% draw figure
    plot(ax,circle_x,circle_y,'b-');
    xlabel('X')
    ylabel('Z')
    hold on;
    plot(ax,0,0,'.');
    plot(ax,Target_Point(1),Target_Point(2),'.','Color','r','MarkerSize',20);
    plot(ax,Way_Points_x,Way_Points_z,'o');
    ax.DataAspectRatio=[1 1 1];
    dy=arrow_length*cosd(Dog_Pos(4));
    dx=arrow_length*sind(Dog_Pos(4));
    quiver(Dog_Pos(2),Dog_Pos(3),dx,dy,'r','LineWidth',0.2,'MaxHeadSize',2);
    plot(Dog_Pos_Record(:,2),Dog_Pos_Record(:,3),'Color','r');
    set(gca,'XDir','reverse');
    xlim(ax,[-3,3]);
    ylim(ax,[-2,2]);
    hold off;
    drawnow;
    %% Stop
    if breakflag == 1
        figure;
        subplot(2,1,1);
        plot(Dog_Pos_Record(:,5),Dog_Pos_Record(:,2));
        xlabel('Time');
        ylabel('X');

        subplot(2,1,2);
        plot(Dog_Pos_Record(:,5),Dog_Pos_Record(:,3));
        xlabel('Time');
        ylabel('Z');

        figure;
        plot(Dog_Pos_Record(:,2),Dog_Pos_Record(:,3));
        xlabel('X');
        ylabel('Z');
        set(gca,'XDir','reverse');
        xlim([-3,3]);
        ylim([-2,2]);
        daspect([1 1 1]);
        break;
    end
    %% time pause
    pause(dt);
end

