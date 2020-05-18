function [closest_point] = check_valid_stop_sign(point, edge, G, x_path)
%@para point - point coordenates
STOP_SIGN_TRESH = 70; %[pixel]

i =  str2double(cell2mat(G.Edges.EndNodes(edge, 1)));
j =  str2double(cell2mat(G.Edges.EndNodes(edge, 2)));

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