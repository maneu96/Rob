
%trajectory generation test using cubic splines and plot the vehicle 
% 
% clear all
% 
% %%%%%%%%%%%%%%%%%%%%%%%%
% % define the reference trajectory
% figure(1)
% % clf
% 
% imshow(imread('ist_map_detail.png'));
% hold on
% %%% starting of the trajectory generation reference stuff
% 
% disp('use the mouse to input via points for the reference trajectory');
% disp('--button 3-- to end the input');
% button = 1;
% k = 1;
% while button==1,
%     [x(k),y(k),button] = ginput(1);
%     plot(x(k),y(k),'r+')
%     k = k + 1;
% end
% drawnow;
% disp([ num2str(k-1), ' points to interpolate from '])
% 
% nodes_coord = [x(:) y(:)]
% %save x; save y;
% %save nodes_coord;

%%
clear all
close all
load x, y;
load nodes_coord
N = length(nodes_coord);
G = digraph;
G = addnode( G, cellstr(string(1:N)))
G.Nodes.X= x';
G.Nodes.Y= y';

G = addedge(G, 14, 19 ,node_dist(G, 14, 19));
G = addedge(G, 19, 20 ,node_dist(G, 19, 20));
G = addedge(G, 20, 21 ,node_dist(G, 20, 21));
G = addedge(G, 21, 22 ,node_dist(G, 21, 22));
G = addedge(G, 21, 24 ,node_dist(G, 21, 24));
G = addedge(G, 22, 23 ,node_dist(G, 22, 23));
G = addedge(G, 23, 24 ,node_dist(G, 23, 24));
G = addedge(G, 24, 25 ,node_dist(G, 24, 25));
G = addedge(G, 25, 13 ,node_dist(G, 25, 13));
G = addedge(G, 25, 26 ,node_dist(G, 25, 26));
G = addedge(G, 26, 27 ,node_dist(G, 26, 27));
G = addedge(G, 26, 29 ,node_dist(G, 26, 29));
G = addedge(G, 29, 13 ,node_dist(G, 29, 13));
G = addedge(G, 13, 12 ,node_dist(G, 13, 12));
G = addedge(G, 12, 10 ,node_dist(G, 12, 10));
G = addedge(G, 10, 14 ,node_dist(G, 10, 14));

imshow(imread('ist_map_detail.png')); hold on
%text( G.Nodes.X+10, G.Nodes.Y+10, G.Nodes.Name); hold
%plot(G.Nodes.X, G.Nodes.Y, 'r+')
plot(G, 'Xdata', G.Nodes.X, 'YData', G.Nodes.Y)

[p ,d] = shortestpath(G, 19, 29)
i=1; step=1;
xq=[G.Nodes.X(p(1))]; vq=[G.Nodes.Y(p(1))];
for i=1:length(p)-1
    %step = (max([G.Nodes.X(p(i)), G.Nodes.X(p(i+1))]) - min([G.Nodes.X(p(i)), G.Nodes.X(p(i+1))]))/3;
    step =5* cos(atan( (G.Nodes.Y(p(i+1))-G.Nodes.Y(p(i)))/(G.Nodes.X(p(i+1))-G.Nodes.X(p(i))) ));
    xqi = min([G.Nodes.X(p(i+1)), G.Nodes.X(p(i))]):step:max([G.Nodes.X(p(i+1)), G.Nodes.X(p(i))]);
    vqi = interp1( [G.Nodes.X(p(i+1)), G.Nodes.X(p(i))], [G.Nodes.Y(p(i+1)), G.Nodes.Y(p(i))], xqi);
    
    if G.Nodes.X(p(i+1))-G.Nodes.X(p(i))<0
        xqi = flip(xqi);
        vqi = flip(vqi);
    end
    
    xq= [xq xqi];
    vq = [vq vqi];
end
figure();
imshow(imread('ist_map_detail.png'))
hold on;
plot( G.Nodes.X,  G.Nodes.Y, 'or');
plot(xq, vq, '.k');

 h = 0.01;
% npt = length(p);        % number of via points, including initial and final
% nvia = [0:1:npt-1];
% csinterp_x = csapi(nvia,G.Nodes.X(p));
% csinterp_y = csapi(nvia,G.Nodes.Y(p));
% time = [0:h:npt-1];
% xx = fnval(csinterp_x, time);
% yy = fnval(csinterp_y, time);



npt = length(xq);        % number of via points, including initial and final
nvia = [0:1:npt-1];
csinterp_x = csapi(nvia,xq);
csinterp_y = csapi(nvia,vq);
time = [0:h:npt-1];
xx_ = fnval(csinterp_x, time);
yy_ = fnval(csinterp_y, time);

figure();
imshow(imread('ist_map_detail.png'))
hold on;
plot( G.Nodes.X,  G.Nodes.Y, 'or');
%plot(xx, yy, 'k--');
plot(xx_, yy_, 'b-');


%%
 p_inicial = [700;900];
%node_dist = vecnorm( [(G.Nodes.X-p_inicial(1))' ;(G.Nodes.Y-p_inicial(2))']);
%[~,node_idx]= min(node_dist);

% IDENTIFICA A EDGE MAIS PERTO DO PONTO INICIAL
for k = 1:size(G.Edges,1)
    i = str2double(cell2mat(G.Edges.EndNodes(k,1))); %start node
    j = str2double(cell2mat(G.Edges.EndNodes(k,2))); %end node
    
    step =5* cos(atan( (G.Nodes.Y(i)-G.Nodes.Y(j))/(G.Nodes.X(i)-G.Nodes.X(j)) ));
    xqi = min([G.Nodes.X(i), G.Nodes.X(j)]):step:max([G.Nodes.X(i), G.Nodes.X(j)]);
    vqi = interp1( [G.Nodes.X(i)-1, G.Nodes.X(j)], [G.Nodes.Y(i), G.Nodes.Y(j)], xqi);
    
    node_dist = vecnorm( [(xqi-p_inicial(1)) ;(vqi-p_inicial(2))]);
    node_val(k)= min(node_dist); % valor minimo a uma edge do par (k) 
end
[~, edge_idx] = min(node_val)

% IDENTIFICA PARA QUAL NODE DEVE "APONTAR"
[~,d1] = shortestpath(G, G.Edges.EndNodes(edge_idx, 1), 29)
[~,d2] = shortestpath(G, G.Edges.EndNodes(edge_idx, 2), 29)
if d1 < d2
   next_node_id =  G.Edges.EndNodes(edge_idx, 1);
else
    next_node_id =  G.Edges.EndNodes(edge_idx, 2);
end

i =  str2double(cell2mat(G.Edges.EndNodes(edge_idx, 1)));
j =  str2double(cell2mat(G.Edges.EndNodes(edge_idx, 2)));

% IDENTIFICAR O PONTO DO PATH + PERTO DO PONTO INICIAL
step =5* cos(atan( (G.Nodes.Y(i)-G.Nodes.Y(j))/(G.Nodes.X(i)-G.Nodes.X(j)) ));
xq_int = min([G.Nodes.X(i), G.Nodes.X(j)]):step:max([G.Nodes.X(i), G.Nodes.X(j)]);
vq_int = interp1( [G.Nodes.X(i)-1, G.Nodes.X(j)], [G.Nodes.Y(i), G.Nodes.Y(j)], xq_int);

node_dist = vecnorm( [(xq_int-p_inicial(1)) ;(vq_int-p_inicial(2))]);
[~, next_point_idx]= min(node_dist); 
next_point = [xq_int(next_point_idx) ; vq_int(next_point_idx)]

aux1 = [xq_int(next_point_idx-1) ; vq_int(next_point_idx-1)] 
aux2 = [xq_int(next_point_idx+1) ; vq_int(next_point_idx+1)] 

aux1 = aux1 - [G.Nodes.X(j) - G.Nodes.Y(j)];
aux2 = aux2 - [G.Nodes.X(j) - G.Nodes.Y(j)];

xqi=[];
vqi=[];
if norm(aux1) < norm(aux2)
    if next_point(1) < G.Nodes.X(i)
        xqi = [xqi xq_int(1:next_point_idx)];
        vqi = [vqi vq_int(1:next_point_idx)];
    else
        xqi = flip([xqi xq_int(1:next_point_idx)]);
        vqi = flip([vqi vq_int(1:next_point_idx)]);
    end
else
    if next_point(1) < G.Nodes.X(i)
        xqi = [xqi xq_int(next_point_idx:end)];
        vqi = [vqi vq_int(next_point_idx:end)];
    else
        xqi = flip([xqi xq_int(next_point_idx:end)]);
        vqi = flip([vqi vq_int(next_point_idx:end)]);
    end
end

% step =5* cos(atan( (G.Nodes.Y(p(i+1))-G.Nodes.Y(p(i)))/(G.Nodes.X(p(i+1))-G.Nodes.X(p(i))) ));
%     xqi = min([G.Nodes.X(p(i+1)), G.Nodes.X(p(i))]):step:max([G.Nodes.X(p(i+1)), G.Nodes.X(p(i))]);
%     vqi = interp1( [G.Nodes.X(p(i+1)), G.Nodes.X(p(i))], [G.Nodes.Y(p(i+1)), G.Nodes.Y(p(i))], xqi);

 
% %%%%%% Pixel to Meters scale
% x_scale = 0.18107;
% disp(['xx scale factor ', num2str(x_scale), ' meters/pixel']);
% 
% y_scale = 0.21394;
% disp(['yy scale factor ', num2str(y_scale), ' meters/pixel']);
% 
% p1 = [2,1]; p2 = [3,5];
% vq = interp1([p1(1), p2(1)], [p1(2), p2(2)], 2:0.1:3)