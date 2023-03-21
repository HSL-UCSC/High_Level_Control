function [result] = Get_Dog_Postion(client,id)

frameData = client.GetLastFrameOfData();

%Get the time stamp
time_stamp = frameData.fTimestamp;

%Get the marker data
Dog_Pos = frameData.RigidBodies(id);
%% Motive coordiate frame (yaw)
% wall wall wall wall wall
%                        0(after)
%                        0
%                        ^ z
%                        |
%                        |
% 90(after)  pi/2 x <----O      -pi/2 270(after)
%
%
%                        0
%                       180(after)
% wall computer wall

%%
if ~isempty(Dog_Pos)
    z=Dog_Pos.z;
    x=Dog_Pos.x;

    q = [Dog_Pos.qx, Dog_Pos.qy, Dog_Pos.qz, Dog_Pos.qw];
    Eul_ang = quat2eul(q);
%% convert yaw
    if abs(Eul_ang(1)) > pi/2
        if -Eul_ang(2) < 0
            yaw = -Eul_ang(2)+2*pi;
        else
            yaw = -Eul_ang(2);
        end
    else
        yaw = pi+Eul_ang(2);
    end
    yaw = yaw*180/pi;

    result = [time_stamp x z yaw];
else
    time_stamp=0;
    z=0;
    x=0;
    yaw=0;
    result = [time_stamp x z yaw];
end

