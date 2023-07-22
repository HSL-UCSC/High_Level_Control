%% Clean
clc;
clear;
close all;

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

real_time=0;
run_time=100;

time=0;

[init_time,x,z,yaw] = Get_Dog_Postion(theClient, Dog_ID); %[time, x, z, yaw]
Dog_Pos_Record = [0,x,z,yaw];
i=1;
figure

%% Main Loop
while true
    [time,x,z,yaw] = Get_Dog_Postion(theClient, Dog_ID); %[time, x, z, yaw]
    real_time=time-init_time;
    time
    real_time
    init_time
    if ~isequal(Dog_Pos_Record(end,:),[real_time,x,z,yaw])
        i=i+1;
        Dog_Pos_Record=[Dog_Pos_Record ; real_time,x,z,yaw];
        real_time = Dog_Pos_Record(i-1,1)-Dog_Pos_Record(1,1);
        if real_time>run_time
            break
        end
    end
    plot(Dog_Pos_Record(:,2),Dog_Pos_Record(:,3))
    xlabel('X');
    ylabel('Z');
    set(gca,'XDir','reverse');
    xlim([-3,3]);
    ylim([-2,2]);
    daspect([1 1 1]);
    drawnow
end

%% figure

% figure;
% plot(Dog_Pos_Record(:,2),Dog_Pos_Record(:,3));
% xlabel('X');
% ylabel('Z');
% set(gca,'XDir','reverse');
% xlim([-3,3]);
% ylim([-2,2]);
% daspect([1 1 1]);
