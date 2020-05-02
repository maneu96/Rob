X=[0;0];
theta=pi/2;
figure;
for i= 2:10000
    %t=(i-1)/1000;
    %plot(X(1),X(2));
    %hold on;
    [X(:,i),theta]=robot(X(:,i-1),theta,pi/4,1,0.001);
end
plot(X(1,:),X(2,:));