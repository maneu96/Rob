
close all;

X=[0;0];
theta=0;
phi=0;
x_path = [4;4];
theta_path = pi/4;
K=1;
i=2;
phi_save = [];
v_save=[];

theta_save=[];

for i=2:20
    [phi,v] = controlador(X(:,i-1),theta, x_path(:,K), theta_path, phi);
    theta_save=[theta_save,theta];
    phi_save = [phi_save, phi];
    v_save = [v_save , v];
    [X(:,i),theta]=robot(X(:,i-1),theta,phi,v,0.01);
end

figure;
scatter(x_path(1,:),x_path(2,:));
hold on;
plot(X(1,:),X(2,:))

figure();
plot(theta_save)

figure();
plot(phi_save);

figure();
plot(v_save);
