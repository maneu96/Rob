function [alpha,v] = controlador(position_rob,theta_rob,position_path, theta_path, alpha_rob, last_v, last_v_phi, delta_t, v_thresh, prev_v_thresh)
%CONTROLADOR Summary of this function goes here
%   Detailed explanation goes here


x_scale = 0.18107;
y_scale = 0.21394;

position_rob = position_rob.* [x_scale; y_scale];
position_path = position_path.* [x_scale; y_scale];

error_vec = [position_path-position_rob; theta_path-theta_rob];

b_error = [cos(theta_rob), sin(theta_rob), 0; -sin(theta_rob), cos(theta_rob), 0; 0, 0, 1]*error_vec;

K_v = 1.2;
K_s = 0.01;
K_i = 0.8;

accel_tresh = 5; %[m^2/s]
%ACCEL_TRESH = accel_tresh * delta_t;
MAX_ACCEL_TRESH = accel_tresh * delta_t;
%v_thresh = %7; % [m/s] = 25,2 [km/h]
V_TRESH = v_thresh * delta_t; 
v = K_v*b_error(1);

W_TRESH = 0.5; % rads/s


W_accel_TRESH = W_TRESH / 0.1; % [demora 0,1 sec até Wmax]
W_accel_TRESH = 0.12;

alpha_dot = K_s*b_error(3)+K_i*b_error(2);
alpha = alpha_rob + alpha_dot;

[alpha, v] = non_linear_block(alpha,v);
alpha_dot = (alpha- last_v_phi)/delta_t;
accel = (v - last_v) / delta_t;

[ACCEL_TRESH] = get_ACCEL(v,last_v, MAX_ACCEL_TRESH, delta_t);

% LIMITE DE ACELERAÇÃO 
if abs(accel) > ACCEL_TRESH
    v = ACCEL_TRESH * delta_t + last_v;
    if sign(accel) ==-1
        v = -ACCEL_TRESH * delta_t + last_v;
    end
end
% LIMITE DE VELOCIDADE 
%if v> V_TRESH
%    v = V_TRESH;
%end
if prev_v_thresh > v_thresh && v>V_TRESH
    v = -ACCEL_TRESH * delta_t + last_v;
    if v< V_TRESH
        v=V_TRESH;
    end
end

if prev_v_thresh < v_thresh && v > last_v
    v = ACCEL_TRESH * delta_t + last_v;
    if v> V_TRESH
        v= V_TRESH;
    end        
end

alpha_accel = (alpha_dot - last_v_phi)/delta_t; % Aceleração angular
% LIMITE DE ACELERAÇÃO ANGULAR
if abs(alpha_accel) > W_accel_TRESH
     alpha_dot = W_accel_TRESH * delta_t +  last_v_phi;
     if sign(alpha_accel) ==-1
         alpha_dot = -W_accel_TRESH * delta_t + last_v_phi;
     end
 end

% LIMITE DE VELOCIDADE ANGULAR
if alpha_dot> W_TRESH;
    alpha_dot = W_TRESH;
end

% LIMITE POSIÇÃO ANGULAR
if abs(alpha)>pi/2.5
    alpha=pi/2.5*sign(alpha);
end

end

