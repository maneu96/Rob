function [theta_path] = get_theta_path(x_path)
%UNTITLED4 Summary of this function goes here
theta_path=[];
for i=2:length(x_path)
    vector_dif = x_path(:,i)-x_path(:,i-1);
    theta_path=[theta_path, atan2(vector_dif(2), vector_dif(1))];
end
theta_path = [theta_path(1) theta_path theta_path(end)];
end

