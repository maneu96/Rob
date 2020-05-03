
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
xq=[]; vq=[];
for i=1:length(p)-1
    step = (max([G.Nodes.X(p(i)), G.Nodes.X(p(i+1))]) - min([G.Nodes.X(p(i)), G.Nodes.X(p(i+1))]))/3;
    xqi = min([G.Nodes.X(p(i)), G.Nodes.X(p(i+1))]):step:max([G.Nodes.X(p(i)), G.Nodes.X(p(i+1))]);
    vqi = interp1( [G.Nodes.X(p(i)), G.Nodes.X(p(i+1))], [G.Nodes.Y(p(i)), G.Nodes.Y(p(i+1))], xqi);
    xq= [xq xqi];
    vq = [vq vqi];
end
figure();
imshow(imread('ist_map_detail.png'))
hold on;
plot( G.Nodes.X,  G.Nodes.Y, 'or');
plot(xq, vq, '.k');

h = 0.01;
npt = length(p);        % number of via points, including initial and final
nvia = [0:1:npt-1];
csinterp_x = csapi(nvia,G.Nodes.X(p));
csinterp_y = csapi(nvia,G.Nodes.Y(p));
time = [0:h:npt-1];
xx = fnval(csinterp_x, time);
yy = fnval(csinterp_y, time);

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
plot(xx, yy, 'k--');
plot(xx_, yy_, 'b-');


% %%%%%% Pixel to Meters scale
% x_scale = 0.18107;
% disp(['xx scale factor ', num2str(x_scale), ' meters/pixel']);
% 
% y_scale = 0.21394;
% disp(['yy scale factor ', num2str(y_scale), ' meters/pixel']);
% 
% p1 = [2,1]; p2 = [3,5];
% vq = interp1([p1(1), p2(1)], [p1(2), p2(2)], 2:0.1:3)