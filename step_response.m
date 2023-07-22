%% Clean
clc;
close all;

%% Robot Dog Network Parameters
% this IP is the vm ip
Robot_Dog_IP = '192.168.123.161';
Robot_Dog_Port = 1145;


%% parameters
run_time = 5;

x_speed = 1.5; %(0.11,1]
z_speed = 0;
%% Robot Dog Command Initialized
Control_Command = zeros(1,11,'single');
%velocity walking
Control_Command(1)=2;
Control_Command(2)=2; %1walk 2run
Control_Command(9)=z_speed;
Control_Command(10)=x_speed;
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

%% initialized
Dog_Pos_Record=[];
Dog_Speed_Record=[];
i=1;
real_time=0;
while true
    [time,x,z,yaw] = Get_Dog_Postion(theClient, Dog_ID); %[time, x, z, yaw]
    if time ~0
        Dog_Pos_Record=[Dog_Pos_Record ; time,x,z,yaw];
        break
    end
end
%% Main Loop
while true
    Robot_Dog(Robot_Dog_IP,Robot_Dog_Port,Control_Command);
    [time,x,z,yaw] = Get_Dog_Postion(theClient, Dog_ID); %[time, x, z, yaw]
    if ~isequal(Dog_Pos_Record(end,:),[time,x,z,yaw])
        i=i+1;
        Dog_Pos_Record=[Dog_Pos_Record ; [time,x,z,yaw]];
        d_dog_pos = Dog_Pos_Record(i,:)-Dog_Pos_Record(i-1,:);
        real_time = Dog_Pos_Record(i-1,1)-Dog_Pos_Record(1,1);
        speed_norm=norm(d_dog_pos(2:3))/d_dog_pos(1);
        Dog_Speed_Record=[Dog_Speed_Record;real_time,speed_norm];
        if real_time>run_time
            Control_Command = zeros(1,11,'single');
            Robot_Dog(Robot_Dog_IP,Robot_Dog_Port,Control_Command);
            if speed_norm < 0.05
                break
            end
        end
    end
end

%% figure
window_size = fix(i/8);
smoothed_v = smooth(Dog_Speed_Record(:,2),window_size);

figure;
plot(Dog_Speed_Record(:,1),Dog_Speed_Record(:,2),'DisplayName','Original');
title('Robot Dog Running Mode 1.5m/s Step Response')
hold on;
plot(Dog_Speed_Record(:,1),smoothed_v,'LineWidth',2,'DisplayName','Smoothed');
yline(norm([x_speed,z_speed]),'DisplayName','Set Speed')
xline(run_time,'r','DisplayName','Stop command');
legend;
xlabel('Time(t)');
ylabel('Speed(m/s)');
hold off;

figure;
plot(Dog_Pos_Record(:,2),Dog_Pos_Record(:,3));
xlabel('X');
ylabel('Z');
set(gca,'XDir','reverse');
xlim([-3,3]);
ylim([-2,2]);
daspect([1 1 1]);
