%% Clean
clc;
close all;

%% Robot Dog Network Parameters
% this IP is the vm ip
Robot_Dog_IP = '192.168.254.134';
Robot_Dog_Port = 1145;


%% parameters
run_time = 5;

x_speed = 1.5; %(0.11,1]
z_speed = 0;
%% Robot Dog Command Initialized
Control_Command = zeros(1,11,'single');
%velocity walking
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
%%
Dog_ID = 1; % Rigid body ID of the drone from Motive
% dance 1
Control_Command(1)=12;

Robot_Dog(Robot_Dog_IP,Robot_Dog_Port,Control_Command);
