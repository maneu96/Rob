close all;

X=[0;0];
theta=0;
phi=0;
x_path = [0:10;0,1,2,4,6,8,10,12,18,40,50];
theta_path = 0;
K=1;
i=2;
phi_save = [];
v_save=[];
while K<4
    %t=(i-1)/1000;
    %plot(X(1),X(2));
    %hold on;
%     if i==5000
%         alpha=pi/2-0.1;
%     end
%     if theta>=pi/2
%         alpha=0;
%     end
    if norm(X(:,i-1)-x_path(:,K))<0.5
        K=K+1;
        figure();
        scatter(x_path(1,:),x_path(2,:));
        hold on;
        plot(X(1,:),X(2,:));
    end

    [phi,v] = controlador(X(:,i-1),theta, x_path(:,K), theta_path, phi);
    phi_save = [phi_save, phi];
    v_save = [v_save , v];
    [X(:,i),theta]=robot(X(:,i-1),theta,phi,v,0.01);
    i=i+1;
end
figure;
plot(x_path(1,:),x_path(2,:));
hold on;
plot(X(1,:),X(2,:));


plot(phi_save);
plot(v_save);
