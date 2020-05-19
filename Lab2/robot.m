function [position,theta] = robot(position,theta,phi,v, delta_t)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
x_scale = 0.18107;
y_scale = 0.21394;
L=2.2;
Y_dot=[ cos(theta) 0; sin(theta) 0; 0 1] * [ v ; (v/L)*tan(phi)];

position = position + (Y_dot(1:2)*delta_t) ./ [x_scale;y_scale];
theta = theta + Y_dot(3)*delta_t;
end

