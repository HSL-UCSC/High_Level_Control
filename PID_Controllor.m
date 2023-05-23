function [Control_Output] = PID_Controllor(K_P,K_I,K_D,D_t,error,integral,previous_error,P_l,I_l,D_l)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%Input: 
% K_P,K_I,K_D, PID parameters
% D_t time
% error
% integral
% previous_error
% P_l,I_l,D_l PID output limit, set 0 to disable.

propostional   = error * K_P;
integral   = integral   + K_I   * error * D_t;
derivative   = K_D   * (error - previous_error)  /D_t;
if P_l ~= 0
    if propostional > P_l
        propostional = P_l;
    elseif propostional < -P_l
        propostional = -P_l;
    end
end
if I_l ~= 0
    if integral > I_l
        integral = I_l;
    elseif integral < -I_l
        integral = -I_l;
    end
end
if D_l ~= 0
    if derivative > D_l
        derivative = D_l;
    elseif derivative < -D_l
        derivative = -D_l;
    end
end
Control_Output = propostional+integral+derivative;
end

