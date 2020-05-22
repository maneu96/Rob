function [xq, vq] = get_path(p_inicial, p_final, G)


[edge_ip] = closest_edge(p_inicial,G); % Closest edge to initial point
[edge_fp] = closest_edge(p_final,G); % Closest edge to final point

%% ENCONTRAR O ERRO(!?)
[path_nodes,edge_i, edge_j] = find_shortestpath(p_inicial, p_final, edge_ip, edge_fp , G); %Initial and final nodes

[xqi_i, vqi_i] = points_to_node(p_inicial, edge_i, G ,0); % Initial point 2 initial node (interpolation)
[xqi_f, vqi_f] = points_to_node(p_final, edge_j, G, 1); % Final node 2 final point (interpolation)

[p ,d] = shortestpath(G, str2double(cell2mat(path_nodes(1))), str2double(cell2mat(path_nodes(2))));
if isempty(p)
    xq =[];
    vq =[];
    return
end

i=1; step=1;
xq=[G.Nodes.X(p(1))]; vq=[G.Nodes.Y(p(1))];
for i=1:length(p)-1
    %step = (max([G.Nodes.X(p(i)), G.Nodes.X(p(i+1))]) - min([G.Nodes.X(p(i)), G.Nodes.X(p(i+1))]))/3;
    step =8* cos(atan( (G.Nodes.Y(p(i+1))-G.Nodes.Y(p(i)))/(G.Nodes.X(p(i+1))-G.Nodes.X(p(i))) ));
    xqi = min([G.Nodes.X(p(i+1)), G.Nodes.X(p(i))]):step:max([G.Nodes.X(p(i+1)), G.Nodes.X(p(i))]);
    vqi = interp1( [G.Nodes.X(p(i+1)), G.Nodes.X(p(i))], [G.Nodes.Y(p(i+1)), G.Nodes.Y(p(i))], xqi);
    
    if G.Nodes.X(p(i+1))-G.Nodes.X(p(i))<0
        xqi = flip(xqi);
        vqi = flip(vqi);
    end
    
    xq= [xq xqi];
    vq = [vq vqi];
end
xq = [xqi_i xq xqi_f p_final(1)];
vq = [vqi_i vq vqi_f p_final(2)];
end

