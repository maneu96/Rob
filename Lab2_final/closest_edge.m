function [edge_idx] = closest_edge(initial_point,G)
%@param G - graph structure

for k = 1:size(G.Edges,1)
    i = str2double(cell2mat(G.Edges.EndNodes(k,1))); %start node
    j = str2double(cell2mat(G.Edges.EndNodes(k,2))); %end node
    step =10* cos(atan( (G.Nodes.Y(i)-G.Nodes.Y(j))/(G.Nodes.X(i)-G.Nodes.X(j)) ));
    xqi = min([G.Nodes.X(i), G.Nodes.X(j)]):step:max([G.Nodes.X(i), G.Nodes.X(j)]);
    vqi = interp1( [G.Nodes.X(i)-1, G.Nodes.X(j)], [G.Nodes.Y(i), G.Nodes.Y(j)], xqi);
    node_dist = vecnorm( [(xqi-initial_point(1)) ;(vqi-initial_point(2))]);
    node_val(k)= min(node_dist); % valor minimo a uma edge do par (k) 

end
[~, edge_idx] = min(node_val);
end

