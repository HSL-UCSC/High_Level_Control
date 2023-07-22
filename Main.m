%% Clean
clc;
close all;

%% Robot Dog Network Parameters
% this IP is the vm ip
Robot_Dog_IP = '192.168.254.134';
Robot_Dog_Port = 1145;

%% Robot Dog Command Initialized
Control_Command = zeros(1,11,'single');
%velocity walking
Control_Command(1)=2;
%% Feedback Control Parameters
dt = 0.05; % time

% Porportional constant on velocity action
K_P_x = 0.6;
K_P_z = 0.6;
K_P_yaw = 0.01;

% Integral
K_I_x = 1.25;
K_I_z = 1.25;
K_I_yaw = 0.002;

% Derivative
K_D_x = 0.08;
K_D_z = 0.08;
K_D_yaw = 0.015;

% limit
propostional_x_limit = 0.7;       % m/s
propostional_z_limit = 0.7;        % m/s
propostional_yaw_limit = 40*pi/180; % rad/s

integral_x_limit = 0.1;
integral_z_limit = 0.1;
integral_yaw_limit = 5*pi/180;

derivative_x_limit = 0.1;
derivative_z_limit = 0.1;
derivative_yaw_limit = 5*pi/180;
%% Control Setting
% MODE
% Mode 1: Dog will go to Target_point.
% Mode 2: Dog will follow Way_Points.

Control_Mode=2;

% Target Point
%[x,z]
Target_Point=[-1 -1];

yaw_set = -2;
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


% THRESHOLD
% Distance Threshold to switch to next way point
Distance_Threshold = 0.15;

% Cricel way points 18
Way_Points_center =[0,0];
Way_Points_radius =1.65;
Way_Points_theta = linspace(0,2*pi,50);
Way_Points_x=Way_Points_center(1)+Way_Points_radius*cos(Way_Points_theta);
Way_Points_z=Way_Points_center(2)+Way_Points_radius*sin(Way_Points_theta);


% %% Path Planning
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

% figure;
% viscircles([random_points_center_x, random_points_center_z], random_points_r,'LineStyle','--','Color','k');
% hold on;
% scatter(random_points(:,1), random_points(:,2),'red','filled');
% axis equal;
% hold off;

% figure;
% plot(Way_Points_x,Way_Points_z,'-o');
% axis equal;



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
Dog_Speed = [];
Dog_Pos_Record_Index = 0;
stop_time = 0;
while true
    [time,x,z,yaw] = Get_Dog_Postion(theClient, Dog_ID);
    if time ~= 0
        init_time = time;
        break
    end
end

while true
    % get position from camera
    [time,x,z,yaw] = Get_Dog_Postion(theClient, Dog_ID); %[time, x, z, yaw]
    Dog_Pos_Record=[Dog_Pos_Record; time, x, z, yaw, time-init_time];
    Dog_Pos_Record_Index = Dog_Pos_Record_Index +1;
    if Dog_Pos_Record_Index > 1
        Dog_Speed_D=Dog_Pos_Record(Dog_Pos_Record_Index,:)-Dog_Pos_Record(Dog_Pos_Record_Index-1,:);
        Dog_Speed = [Dog_Speed;Dog_Speed_D(2)/Dog_Speed_D(1),Dog_Speed_D(3)/Dog_Speed_D(1),norm(Dog_Speed_D(2)/Dog_Speed_D(1),Dog_Speed_D(3)/Dog_Speed_D(1)),time-init_time];
        
    end
    if Control_Mode == 2
        Target_Point = [Way_Points_x(Way_Point_index) Way_Points_z(Way_Point_index)];
    end
    % Feedback control
    
    % Calculate Distance
    Point_Dog = [x z];  %[x,z]
    Vector_PD_TP = Target_Point-Point_Dog; % Get vector
    Norm_Vector = norm(Vector_PD_TP);      % Calculate norm
    
    % rotation matrix to turn vector to robot dog frame
    Rotation_matrix = [cosd(yaw), -sind(yaw) ; sind(yaw),cosd(yaw) ];
    
    
    %vector in robot dog frame = error_x error_z
    Vector_rotated = Rotation_matrix*Vector_PD_TP';
    
    Error_Yaw=Yaw_Controllor(yaw_set,yaw,Vector_PD_TP);
    % PID
    Control_x=PID_Controllor(K_P_x,K_I_x,K_D_x,dt,Vector_rotated(1),integral_x,previous_error_x,propostional_x_limit,integral_x_limit,derivative_x_limit);
    Control_z=PID_Controllor(K_P_z,K_I_z,K_D_z,dt,Vector_rotated(2),integral_z,previous_error_z,propostional_z_limit,integral_z_limit,derivative_z_limit);
    Control_yaw=PID_Controllor(K_P_yaw,K_I_yaw,K_D_yaw,dt,Error_Yaw,integral_yaw,previous_error_yaw,propostional_yaw_limit,integral_yaw_limit,derivative_yaw_limit);
    
    propostional_x   = Vector_rotated(1) * K_P_x;
    propostional_z   = Vector_rotated(2) * K_P_z;
    propostional_yaw = Error_Yaw*pi/180 * K_P_yaw;
    
    % set command
    Control_Command(10) = Control_x;   %x
    Control_Command(9)  = Control_z;   %z
    Control_Command(11) = Control_yaw; %yaw
    if Norm_Vector < Distance_Threshold
        if Control_Mode == 1
            if stop_time == 0
                stop_time = time + 5;
            end
            if time > stop_time
                break
            end
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
    %     disp(Control_Command);
    %     fprintf("hi\n");
    % send command to vitrual machine
    Robot_Dog(Robot_Dog_IP,Robot_Dog_Port,Control_Command);
    % draw figure
    plot(ax,circle_x,circle_y,'b-');
    xlabel('X')
    ylabel('Z')
    hold on;
    plot(ax,0,0,'.');
    plot(ax,Target_Point(1),Target_Point(2),'.','Color','r','MarkerSize',20);
    plot(ax,Way_Points_x,Way_Points_z,'o');
    ax.DataAspectRatio=[1 1 1];
    dy=arrow_length*cosd(yaw);
    dx=arrow_length*sind(yaw);
    quiver(x,z,dx,dy,'r','LineWidth',0.2,'MaxHeadSize',2);
    plot(Dog_Pos_Record(:,2),Dog_Pos_Record(:,3),'Color','r');
    set(gca,'XDir','reverse');
    xlim(ax,[-3,3]);
    ylim(ax,[-2,2]);
    hold off;
    drawnow;
    % Stop
    if breakflag == 1
        break;
    end
    
end
%%
figure;
subplot(2,1,1);
plot(Dog_Pos_Record(:,5),Dog_Pos_Record(:,2),'DisplayName','X');
title('Robot Dog X')
yline(Target_Point(1),'DisplayName','Target X','Color','r')
legend;
xlabel('Time');
ylabel('X');

subplot(2,1,2);
plot(Dog_Pos_Record(:,5),Dog_Pos_Record(:,3),'DisplayName','Z');
title('Robot Dog Z')
yline(Target_Point(1),'DisplayName','Target Z','Color','r')
legend;
xlabel('Time');
ylabel('Z');

figure;
plot(Dog_Pos_Record(:,5),Dog_Pos_Record(:,4),'DisplayName','Yaw');
yline(yaw_set,'DisplayName','Target Yaw','Color','r')
legend;
xlabel('Time');
ylabel('Yaw');
title('Robot Dog Yaw')

figure;
plot(Dog_Pos_Record(:,2),Dog_Pos_Record(:,3));
xlabel('X');
ylabel('Z');
set(gca,'XDir','reverse');
xlim([-3,3]);
ylim([-2,2]);
daspect([1 1 1]);

figure;
plot(Dog_Speed(:,4),Dog_Speed(:,3));
xlabel('Time');
ylabel('Speed (m/s)');
title('Robot Dog Norm Speed')



