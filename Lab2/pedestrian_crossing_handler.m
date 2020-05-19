function [] = pedestrian_crossing_handler(pedestrian_crossing_signs,pedestrian_crossing_times, X, x_path, i, delta_t)
closest_point=[];
if isempty(pedestrian_crossing_signs)
    %????????
    return 
end
for i = 1 : length(pedestrian_crossing_signs)
    [aux] = check_valid_traffic_sign(pedestrian_crossing_signs(:,i), x_path);
    if ~isempty(aux)
        closest_point=[closest_point aux];
    end
end    

%%% ADAPTAR ULTIMA PARTE DO COD. PARA PEDESTRIAN

[~, X_idx]=min( vecnorm(x_path - X));
for i =1:size(closest_point,2)
    cp_x_idx =find(closest_point(1,i) == x_path(1,:));
    cp_y_idx =find(closest_point(2,i) == x_path(2,:));
    cp_idx = union(cp_x_idx,cp_y_idx);
    idx_dif(i) = cp_idx(1) - X_idx;
end
next_speed_sign_path= min(idx_dif((find( idx_dif >=0)))); % INDICE EM X_PATH em rela��o ao X
next_speed_sign_cp= min((find( idx_dif >=0))); % INDICE EM CLOSEST_POINT

dist_int=0;


for i=X_idx+1:X_idx+next_speed_sign_path
    dist_int = dist_int+ norm(x_path(:,i)-x_path(:,i-1));
end


if SIGNAL_DISTANCE_VISIBILITY>dist_int
    if current_speed_limit > closest_point(3, next_speed_sign_cp)
        speed_limit = closest_point(3, next_speed_sign_cp);
    elseif abs(next_speed_sign_path) <=1
        speed_limit = closest_point(3, next_speed_sign_cp);
    else
         speed_limit=current_speed_limit;
    end 
else
    speed_limit=current_speed_limit;
end
    
    

end
