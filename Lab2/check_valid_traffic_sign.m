function [closest_point] = check_valid_traffic_sign(point, x_path)
%@para point - point coordenates
STOP_SIGN_TRESH = 70; %[pixel]

% IDENTIFICAR O PONTO DO PATH + PERTO DO PONTO INICIAL
xq_int = x_path(1,:);
vq_int = x_path(2,:);

node_dist = vecnorm( [(xq_int-point(1)) ;(vq_int-point(2))]);
[norm, point_idx]= min(node_dist); 

if norm > STOP_SIGN_TRESH
    closest_point =[];
else
    closest_point = [xq_int(point_idx) ; vq_int(point_idx)];
end
end