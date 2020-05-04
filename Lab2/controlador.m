function [alpha,v,error_vec,b_error] = controlador(position_rob,theta_rob,position_path, theta_path, alpha_rob)
%CONTROLADOR Summary of this function goes here
%   Detailed explanation goes here

error_vec = [position_path-position_rob; theta_path-theta_rob];

b_error = [cos(theta_rob), sin(theta_rob), 0; -sin(theta_rob), cos(theta_rob), 0; 0, 0, 1]*error_vec;

K_v = 0.5;
K_s = 0.1;
K_i = 1;

v = K_v*b_error(1);

% if norm(error_vec(1:2))>0.2 && abs(v)<0.1 || v<0
%     v = 0.5;
% end

alpha_dot = K_s*b_error(3)+K_i*b_error(2);
alpha = alpha_rob + alpha_dot;

if abs(alpha)>pi/2.5
    alpha=pi/2.5*sign(alpha);
end

end

