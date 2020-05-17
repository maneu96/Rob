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

v_save=[0];
theta_save=[];
error_save=[];
b_error_save=[];
theta_path=[];
for i=2:length(x_path)
    vector_dif = x_path(:,i)-x_path(:,i-1);
    theta_path=[theta_path, atan2(vector_dif(2), vector_dif(1))];
end
phi_save = [theta_path(1)];

i=2;
theta_path = [theta_path(1) theta_path theta_path(end)];

X=[x_path(1,1);x_path(2,1)];
theta=theta_path(1);

M = 810; %[Kg]
delta_t =0.1;
P0 = 0; v=0;v_phi=0;
while 1%K<=length(x_path)


     if (norm(X(:,i-1)-x_path(:,K))<8) %||...
%             ((abs(error_save(1,end))<=0.1) && (abs(theta)<=0.1||(abs(theta)<=pi+0.1 && abs(theta)>=pi-0.1))) ||...
%             ((abs(error_save(2,end))<=0.1) && (abs(theta)<=pi/2+0.1 || abs(theta)>=pi/2-0.1))
        
        
        if K==length(x_path) 
            if norm(X(:,i-1)-x_path(:,end))<=2
                break
            end
        else
            K=K+1
        end
        %figure();
        %scatter(x_path(1,:),x_path(2,:));
        %hold on;
        %plot(X(1,:),X(2,:));
     end

    [phi,v,error_vec,b_error] = controlador(X(:,i-1),theta, x_path(:,K), theta_path(K+1), phi, v,v_phi, delta_t);
    
    theta_save=[theta_save,theta];
    phi_save = [phi_save, phi];
    v_save = [v_save , v];
    a_save(i) = (v - v_save(end-1)) / delta_t;
    v_phi = phi-phi_save(end-1);
    
    %energy_spent(i) = (M*a_save(i) + P0)* v * delta_t;
    
    error_save=[error_save, error_vec];
    b_error_save = [b_error_save, b_error];
    [X(:,i),theta]=robot(X(:,i-1),theta,phi,v, delta_t);
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

%%

plot( (1:length(a_save))*delta_t, a_save/delta_t);
hold on
plot( (1:length(a_save))*delta_t, v_save/delta_t);

figure()
plot( (1:length(phi_save))*delta_t, phi_save);

