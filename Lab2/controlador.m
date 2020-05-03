function [alpha,v] = controlador(position_rob,theta_rob,position_path, theta_path, alpha_rob)
%CONTROLADOR Summary of this function goes here
%   Detailed explanation goes here

error_vec = [position_path-position_rob; theta_path-theta_rob];

b_error = [cos(theta_rob), sin(theta_rob), 0; -sin(theta_rob), cos(theta_rob), 0; 0, 0, 1]*error_vec;

K_v = 100;
K_s = 100;
K_i = 1;

v = K_v*b_error(1);

alpha_dot = K_s*b_error(3)+K_i*b_error(2);
alpha = alpha_rob + alpha_dot;

if abs(alpha)>pi/8
    alpha=pi/8*sign(alpha);
end

end

