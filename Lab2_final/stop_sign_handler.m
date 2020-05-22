function [paths] = stop_sign_handler(stop_signs, x_path)
closest_point=[];
if isempty(stop_signs)
    paths = {x_path};
    return 
end
for i = 1 : size(stop_signs,2)
    [aux] = check_valid_traffic_sign(stop_signs(:,i), x_path);
    if ~isempty(aux)
        closest_point=[closest_point aux];
    end
end

paths = {};
paths_idx = {};
idx =1;
for i = 2:size(closest_point,2)+1
    idx_x = find(x_path(1,:) == closest_point(1,i-1));
    idx_y = find(x_path(2,:) == closest_point(2,i-1));
    auxx = union(idx_x, idx_y);
    idx(i) = auxx(1);
    paths(i-1) = {x_path(:, idx(i-1):idx(i))};
end
paths(end+1) = {x_path(:, idx(end):end)};

end

