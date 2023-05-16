function Robot_Dog(Robot_Dog_IP,Robot_Dog_Port,Control_Data)
% Send command to robot dog

% Create a UDP object
Robot_Dog_UDP = udpport("LocalHost",'192.168.12.136');
% Transform single float to uint8
Control_Data_Uint8 = typecast(Control_Data,'uint8');
% Send Command
write(Robot_Dog_UDP,Control_Data_Uint8,Robot_Dog_IP,Robot_Dog_Port);
end

