function [path_nodes] = find_shortestpath(initial_point,final_point, edge_ip, edge_fp , G)
%UNTITLED2 Summary of this function goes here
c=1;
node_pairs=[];
edge_ip
aux_pair = flip([str2double(cell2mat(G.Edges.EndNodes(edge_ip,1))) str2double(cell2mat(G.Edges.EndNodes(edge_ip,2)))]);
edge_pair = findedge(G,aux_pair(1), aux_pair(2) );
if edge_pair ==0
    I_SIZE = 2;
else
    I_SIZE = 1;
end

aux_pair = flip([str2double(cell2mat(G.Edges.EndNodes(edge_fp,1))) str2double(cell2mat(G.Edges.EndNodes(edge_fp,2)))]);
edge_pair = findedge(G,aux_pair(1), aux_pair(2) );
if edge_pair ==0
    J_SIZE = 1;
else
    J_SIZE = 2;
end

for i =I_SIZE:2
    for j=1:J_SIZE
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

