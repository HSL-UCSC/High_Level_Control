function [error_yaw_command,yaw_ref] = Yaw_Controllor(yaw_set,Dog_yaw,Vector)
%YAW_CONTROLLOR Summary of this function goes here
%   Detailed explanation goes here
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
if yaw_set == -1 % not control yaw
    error_yaw_command=0;
    yaw_ref = 0;
elseif yaw_set == -2 % control yaw to motion direction
    angle_r = atan2(Vector(1),Vector(2));
    yaw_set_mode2 = rad2deg(angle_r);
    if yaw_set_mode2 < 0
        yaw_set_mode2 = yaw_set_mode2+360;
    end
    error_yaw = yaw_set_mode2-Dog_yaw;
    yaw_ref=yaw_set_mode2;
    if error_yaw<-180
        error_yaw_command=(360+error_yaw);
    elseif error_yaw > 180
        error_yaw_command=error_yaw-360;
    else
        error_yaw_command=error_yaw;
    end
elseif yaw_set >=0 && yaw_set<360  % control yaw
    error_yaw = yaw_set-Dog_yaw;
    if error_yaw<-180
        error_yaw_command=(360+error_yaw);
    elseif error_yaw > 180
        error_yaw_command=error_yaw-360;
    else
        error_yaw_command=error_yaw;
    end
    yaw_ref = yaw_set;
end

end

