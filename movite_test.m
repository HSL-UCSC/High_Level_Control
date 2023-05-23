%% Clean
clc;
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
Dog_Pos_Record=[];
i=0;
real_time=0;
%% Main Loop
while true
   
    [Dog_Pos] = Get_Dog_Postion(theClient, Dog_ID); %[time, x, z, yaw]
    if i<1
        i=i+1;
        Dog_Pos_Record=[Dog_Pos_Record ; Dog_Pos];
    else
        if ~isequal(Dog_Pos_Record(end,:),Dog_Pos)
            i=i+1;
            disp([Dog_Pos(2),Dog_Pos(3),Dog_Pos(4)]);
            Dog_Pos_Record=[Dog_Pos_Record ; Dog_Pos];
        end
    end
end


