function [d] = node_dist(G,idx1, idx2)
%%% Receives a graph G and returns a distance d, between node 1 and node 2
    d = pdist([G.Nodes.X(idx1), G.Nodes.Y(idx1);G.Nodes.X(idx2), G.Nodes.Y(idx2)],'euclidean');
end

