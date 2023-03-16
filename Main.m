%% Clean
clc; 
close all;

% this IP is the vm ip
Robot_Dog_IP = '192.168.12.184';
Robot_Dog_Port = 1145;

Control_Command = zeros(1,11,'single');
%% Motive coordiate frame
% wall wall wall wall wall
%        ^ z
%        |
%        |
% x <----O y(pointing up)
% wall computer wall


%% System Parameters

K_P = 0.5; % Porportional constant on velocity action
K_I = 1;  % Integral
K_D = 1;
% constant on velocity action
K_Y = pi/180; % Proportional constant on angle action


%% Instantiate client object to run Motive API commands
% https://optitrack.com/software/natnet-sdk/

% Create Motive client object
dllPath = fullfile('d:','StDroneControl','NatNetSDK','lib','x64','NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath); % Add API function calls
theClient = NatNetML.NatNetClientML(0);

% Create connection to localhost, data is now being streamed through client object
HostIP = '127.0.0.1';
theClient.Initialize(HostIP, HostIP); 

Dog_ID = 1; % Rigid body ID of the drone from Motive

Target_Point=[0 0];
yaw_set = 0;

%% figure for movtion track
fig = figure();
ax = axes('Parent',fig);
line_handle = plot(ax,Dog_Pos(3),Dog_Pos(2),'o');

xlabel('X')
ylabel('Z')

arrow_length=0.2;


%%
% while true
%     [Dog_Pos] = Get_Dog_Postion(theClient, Dog_ID);
% 
% end

%velocity walking
Control_Command(1)=2; 
%% Robot dog command
%
%     +(11) +(9)  -(11)
%     +(10)  dog  -(10)
%           -(9)
%
%%
while true
    [Dog_Pos] = Get_Dog_Postion(theClient, Dog_ID); %[time, z, x, yaw]
    
    error_Z = Target_Point(1)-Dog_Pos(2);
    error_X = -Target_Point(2)-Dog_Pos(3);
    
    disp(error_X);

    error_yaw = yaw_set-Dog_Pos(4);

    if abs(error_yaw)>180
        error_yaw=(360+error_yaw);
    end

    %disp(error_yaw*K_Y);
    Control_Command(11) = error_yaw*K_Y;

    if abs(error_yaw)<10
        Control_Command(9) = error_Z*K_P;
        Control_Command(10) = error_X*K_P;
    end

    

    Robot_Dog(Robot_Dog_IP,Robot_Dog_Port,Control_Command);

    pause(0.02);

    dy=arrow_length*cosd(Dog_Pos(4));
    dx=arrow_length*sind(Dog_Pos(4));
    quiver(Dog_Pos(3),Dog_Pos(2),dx,dy,'r','LineWidth',0.2,'MaxHeadSize',2);
    set(gca,'XDir','reverse');
    xlim(ax,[-2,2]);
    ylim(ax,[-2,2]);
    %set(line_handle, 'XData',Dog_Pos(3),'YData',Dog_Pos(2))
    drawnow;
end



