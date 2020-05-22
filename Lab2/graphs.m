
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
% clear all
% close all
 load x, y;
 load nodes_coord
N = length(nodes_coord);
G = digraph;
G = addnode( G, cellstr(string(1:N+2)))
G.Nodes.X= [x 955 1220]';
G.Nodes.Y= [y 13 75]';
G.Nodes.X(27) = 753;
G.Nodes.Y(27) = 1720;

G.Nodes.X(28) = 792;
G.Nodes.Y(28) = 1720;

G.Nodes.X(34) = 20;
G.Nodes.Y(34) = 144;

G.Nodes.X(23) = 416;
G.Nodes.Y(23) = 1460;

G.Nodes.X(22) = 358;
G.Nodes.Y(22) = 1460;

% G.Nodes.X(end+1) = 955;
% G.Nodes.Y(end +1) = 13;
% 
% G.Nodes.X(end+1) = 1220;
% G.Nodes.Y(end +1) = 75;

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
G = addedge(G, 10, 14 ,node_dist(G, 10, 14));
G = addedge(G, 15, 14 ,node_dist(G, 15, 14));
G = addedge(G, 28, 29 ,node_dist(G, 28, 29));
G = addedge(G, 29, 30 ,node_dist(G, 29, 30));
G = addedge(G, 30, 31 ,node_dist(G, 30, 31));
G = addedge(G, 31, 32 ,node_dist(G, 31, 32));
G = addedge(G, 32, 33 ,node_dist(G, 32, 33));
G = addedge(G, 10, 12 ,node_dist(G, 10, 12));
G = addedge(G, 12, 11 ,node_dist(G, 12, 11));
G = addedge(G, 11, 10 ,node_dist(G, 11, 10));
G = addedge(G, 3, 10 ,node_dist(G, 3, 10));
G = addedge(G, 13, 25 ,node_dist(G, 13, 25));
G = addedge(G, 11, 4 ,node_dist(G, 11, 4));
G = addedge(G, 3, 4 ,node_dist(G, 3, 4));
G = addedge(G, 4, 5 ,node_dist(G, 4, 5));
G = addedge(G, 5, 6 ,node_dist(G, 5, 6));
G = addedge(G, 6, 7 ,node_dist(G, 6, 7));
G = addedge(G, 7, 8 ,node_dist(G, 7, 8));
G = addedge(G, 8, 9 ,node_dist(G, 8, 9));
G = addedge(G, 1, 3 ,node_dist(G, 1, 3));
G = addedge(G, 4, 2 ,node_dist(G, 4, 2));
G = addedge(G, 1, 2 ,node_dist(G, 1, 2));
G = addedge(G, 34, 1 ,node_dist(G, 34, 1));
G = addedge(G, 15, 16 ,node_dist(G, 15, 16));
G = addedge(G, 16, 15 ,node_dist(G, 16, 15));
G = addedge(G, 17, 16,node_dist(G, 17, 16));
G = addedge(G, 16, 17 ,node_dist(G, 16, 17));
G = addedge(G, 17, 18 ,node_dist(G, 17, 18));
G = addedge(G, 18, 17 ,node_dist(G, 18, 17));
G = addedge(G, 12, 13 ,node_dist(G, 12, 13));
G = addedge(G, 14, 15 ,node_dist(G, 14, 15));
G = addedge(G, 33, 32 ,node_dist(G, 33, 32));
G = addedge(G, 32, 31 ,node_dist(G, 32, 31));
G = addedge(G, 31, 30 ,node_dist(G, 31, 30));
G = addedge(G, 30, 29 ,node_dist(G, 30, 29));
G = addedge(G, 9, 8 ,node_dist(G, 9, 8));
G = addedge(G, 8, 7 ,node_dist(G, 8, 7));
G = addedge(G, 7, 6 ,node_dist(G, 7, 6));
G = addedge(G, 6, 5 ,node_dist(G, 6, 5));
G = addedge(G, 5, 4 ,node_dist(G, 5, 4));
G = addedge(G, 2, 35 ,node_dist(G, 2, 35));
G = addedge(G, 35, 36 ,node_dist(G, 35, 36));
G = addedge(G, 4, 3 ,node_dist(G, 4, 3));

save('G', 'G');



imshow(imread('ist_map_detail.png')); hold on
%text( G.Nodes.X+10, G.Nodes.Y+10, G.Nodes.Name); hold
%plot(G.Nodes.X, G.Nodes.Y, 'r+')
plot(G, 'Xdata', G.Nodes.X, 'YData', G.Nodes.Y)

p_inicial = [700;900]; %USER INPUT %%%%%%%%%
p_inicial = [340;900]; %USER INPUT %%%%%%%%%
p_final = [760; 1560]; %USER INPUT %%%%%%%%%


[xq, vq] = get_path(p_inicial, p_final, G); %Gets all reference points in the path


figure();
imshow(imread('ist_map_detail.png'))
hold on;
plot( G.Nodes.X,  G.Nodes.Y, 'or');
plot(xq, vq, '.k');

%  h = 0.01;
% npt = length(p);        % number of via points, including initial and final
% nvia = [0:1:npt-1];
% csinterp_x = csapi(nvia,G.Nodes.X(p));
% csinterp_y = csapi(nvia,G.Nodes.Y(p));
% time = [0:h:npt-1];
% xx = fnval(csinterp_x, time);
% yy = fnval(csinterp_y, time);



% npt = length(xq);        % number of via points, including initial and final
% nvia = [0:1:npt-1];
% csinterp_x = csapi(nvia,xq);
% csinterp_y = csapi(nvia,vq);
% time = [0:h:npt-1];
% xx_ = fnval(csinterp_x, time);
% yy_ = fnval(csinterp_y, time);
% 
% figure();
% imshow(imread('ist_map_detail.png'))
% hold on;
% plot( G.Nodes.X,  G.Nodes.Y, 'or');
% %plot(xx, yy, 'k--');
% plot(xx_, yy_, 'b-');


%%


 
% %%%%%% Pixel to Meters scale
% x_scale = 0.18107;
% disp(['xx scale factor ', num2str(x_scale), ' meters/pixel']);
% 
% y_scale = 0.21394;
% disp(['yy scale factor ', num2str(y_scale), ' meters/pixel']);
% 
% p1 = [2,1]; p2 = [3,5];
% vq = interp1([p1(1), p2(1)], [p1(2), p2(2)], 2:0.1:3)