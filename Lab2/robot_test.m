close all;
clear all;

%X=[0;0];
%theta=pi/4;
phi=0;
% x_path = [0:10;0,1,2,3,4,5,4,5,4,5,4];
% x_path = [0:10;5*sin(0:0.5:5)];
% load x_path
load G
p_inicial = [700;900]; %USER INPUT %%%%%%%%%
p_inicial = [340;900]; %USER INPUT %%%%%%%%%
p_final = [760; 1560]; %USER INPUT %%%%%%%%%
[xq, vq] = get_path(p_inicial, p_final, G); %Gets all reference points in the path
x_path = [xq; vq];

K=1;
phi_save = [];
v_save=[];
theta_save=[];
error_save=[];
b_error_save=[];
theta_path=[];
for i=2:length(x_path)
    vector_dif = x_path(:,i)-x_path(:,i-1);
    theta_path=[theta_path, atan2(vector_dif(2), vector_dif(1))];
end

i=2;
theta_path = [theta_path, theta_path(end)];

X=[x_path(1,1);x_path(2,1)];
theta=theta_path(1);


while K<=length(x_path)
    %t=(i-1)/1000;
    %plot(X(1),X(2));
    %hold on;
%     if i==5000
%         alpha=pi/2-0.1;
%     end
%     if theta>=pi/2
%         alpha=0;
%     end
     if (norm(X(:,i-1)-x_path(:,K))<3) %||...
%             ((abs(error_save(1,end))<=0.1) && (abs(theta)<=0.1||(abs(theta)<=pi+0.1 && abs(theta)>=pi-0.1))) ||...
%             ((abs(error_save(2,end))<=0.1) && (abs(theta)<=pi/2+0.1 || abs(theta)>=pi/2-0.1))
        
        K=K+1
        %figure();
        %scatter(x_path(1,:),x_path(2,:));
        %hold on;
        %plot(X(1,:),X(2,:));
    end

    [phi,v,error_vec,b_error] = controlador(X(:,i-1),theta, x_path(:,K), theta_path(K+1), phi);
    theta_save=[theta_save,theta];
    phi_save = [phi_save, phi];
    v_save = [v_save , v];
    error_save=[error_save, error_vec];
    b_error_save = [b_error_save, b_error];
    [X(:,i),theta]=robot(X(:,i-1),theta,phi,v,0.01);
    i=i+1;
end

figure;
imshow(imread('ist_map_detail.png')); hold on
scatter(x_path(1,:),x_path(2,:));
hold on;
plot(X(1,:),X(2,:));

figure();
plot(theta_save)

figure();
plot(phi_save);

figure();
plot(v_save);

figure();
plot(error_save(1,:));

figure();
plot(error_save(2,:));

figure();
plot(b_error_save(1,:));