function [path_nodes] = find_shortestpath(initial_point,final_point, edge_ip, edge_fp , G)
%UNTITLED2 Summary of this function goes here
c=1;
node_pairs=[];
for i =1:2
    for j=1:2
        [~,d1] = shortestpath(G, G.Edges.EndNodes(edge_ip, i), G.Edges.EndNodes(edge_fp, j));
        node_idx = G.Edges.EndNodes(edge_ip, i);
        node_coord = [G.Nodes.X(str2double(cell2mat(node_idx))); G.Nodes.Y(str2double(cell2mat(node_idx)))];
        node_dist_ip = norm( initial_point-node_coord);
        
        node_idx = G.Edges.EndNodes(edge_fp, j);
        node_coord = [G.Nodes.X(str2double(cell2mat(node_idx))); G.Nodes.Y(str2double(cell2mat(node_idx)))];
        node_dist_fp = norm( final_point-node_coord);
        
        d(c) = d1 + node_dist_ip + node_dist_fp;
        node_pairs = [node_pairs, [G.Edges.EndNodes(edge_ip, i);G.Edges.EndNodes(edge_fp, j)] ];
        c=c+1;  
    end
end
[~, idx] = min(d);
path_nodes = node_pairs(:,idx);

end

