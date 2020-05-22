function [speed_limit] = pedestrian_traffic_sign_handler(pedestrian_crossing_signs,X, x_path, current_speed_limit, prev_speed_limit)
SPEED_LIMIT = 2.77; %[m/s] = 10 [km/h]

pedestrian_crossing_signs = [pedestrian_crossing_signs; SPEED_LIMIT*ones(1,size(pedestrian_crossing_signs,2)) ];

[speed_limit] = speed_limit_handler(pedestrian_crossing_signs, X, x_path, current_speed_limit);

if speed_limit > current_speed_limit
    speed_limit = current_speed_limit;
end
%%%%%%----%%%
closest_point=[];
for i = 1 : size(pedestrian_crossing_signs,2)
    [aux] = check_valid_traffic_sign(pedestrian_crossing_signs(:,i), x_path);

    if ~isempty(aux)
        closest_point=[closest_point aux];
    end
end

if isempty(closest_point)
    speed_limit = current_speed_limit;
    return
end

[~, X_idx]=min( vecnorm(x_path - X));
for i =1:size(closest_point,2)
    cp_x_idx =find(closest_point(1,i) == x_path(1,:));
    cp_y_idx =find(closest_point(2,i) == x_path(2,:));
    cp_idx = union(cp_x_idx,cp_y_idx);
    idx_dif(i) = cp_idx(1) - X_idx;
end
next_speed_sign_path= min(idx_dif((find( idx_dif >=0)))); % INDICE EM X_PATH em relação ao X

if next_speed_sign_path==0
    speed_limit = prev_speed_limit;
end
end

