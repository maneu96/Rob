function [xqi, vqi] = points_to_node(point, edge, G, flag)
%@para point - point coordenates
% @param flag = 0 -> point 2 node; 
%             = 1 -> node 2 point

if flag == 0
    i =  str2double(cell2mat(G.Edges.EndNodes(edge, 1)));
    j =  str2double(cell2mat(G.Edges.EndNodes(edge, 2)));
else
    j =  str2double(cell2mat(G.Edges.EndNodes(edge, 1)));
    i =  str2double(cell2mat(G.Edges.EndNodes(edge, 2)));
end

% IDENTIFICAR O PONTO DO PATH + PERTO DO PONTO INICIAL
step =5* cos(atan( (G.Nodes.Y(i)-G.Nodes.Y(j))/(G.Nodes.X(i)-G.Nodes.X(j)) ));
xq_int = min([G.Nodes.X(i), G.Nodes.X(j)]):step:max([G.Nodes.X(i), G.Nodes.X(j)]);
vq_int = interp1( [G.Nodes.X(i)-1, G.Nodes.X(j)], [G.Nodes.Y(i), G.Nodes.Y(j)], xq_int);

node_dist = vecnorm( [(xq_int-point(1)) ;(vq_int-point(2))]);
[~, next_point_idx]= min(node_dist); 
next_point = [xq_int(next_point_idx) ; vq_int(next_point_idx)];

if next_point_idx == 1
    aux1 = [xq_int(next_point_idx) ; vq_int(next_point_idx)];
    aux2 = [xq_int(next_point_idx+1) ; vq_int(next_point_idx+1)];
elseif next_point_idx == size(xq_int,2)
    aux1 = [xq_int(next_point_idx-1) ; vq_int(next_point_idx-1)];
    aux2 = [xq_int(next_point_idx) ; vq_int(next_point_idx)];
else
    aux1 = [xq_int(next_point_idx-1) ; vq_int(next_point_idx-1)];
    aux2 = [xq_int(next_point_idx+1) ; vq_int(next_point_idx+1)];
end
    


aux1 = aux1 - [G.Nodes.X(j) ; G.Nodes.Y(j)];
aux2 = aux2 - [G.Nodes.X(j) ; G.Nodes.Y(j)];


if norm(aux1) < norm(aux2)
    if next_point(1) < G.Nodes.X(i)
        xqi = flip(xq_int(1:next_point_idx));
        vqi = flip(vq_int(1:next_point_idx));
    else
        xqi = (xq_int(1:next_point_idx));
        vqi = (vq_int(1:next_point_idx));
    end
else
    if next_point(1) < G.Nodes.X(i)
        xqi =  flip(xq_int(next_point_idx:end));
        vqi =  flip(vq_int(next_point_idx:end));
    else
        xqi = ( xq_int(next_point_idx:end));
        vqi = ( vq_int(next_point_idx:end));
    end
end
if flag
    xqi = flip(xqi);
    vqi = flip(vqi);
end
end

