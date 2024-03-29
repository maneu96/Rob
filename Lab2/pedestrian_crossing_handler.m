function [K] = pedestrian_crossing_handler(sim_time, pedestrian_crossing_pos, pedestrian_crossing_times, X, x_path, v, delta_t, K )
PEDESTRIAN_CROSSING_VISIBILITY = 20; %[m]
SAFETY_TIME_TRESH = 2; %[s]
V_STOP_TRESH = 0.3; %[m/s]
if isempty(pedestrian_crossing_pos)
   return 
end

x_scale = 0.18107;
y_scale = 0.21394;

valid_crossings_idx = find((pedestrian_crossing_times(1,:)+ pedestrian_crossing_times(2,:)) - sim_time >0);
if isempty(valid_crossings_idx)
   return 
end

valid_crossings_pos = pedestrian_crossing_pos(:,valid_crossings_idx(1));
valid_crossings_times = pedestrian_crossing_times(:,valid_crossings_idx(1));

closest_point = check_valid_traffic_sign(valid_crossings_pos, x_path) ;
if isempty(closest_point)
   return 
end
[~, X_idx]=min( vecnorm(x_path - X));

cp_x_idx =find(closest_point(1) == x_path(1,:));
cp_y_idx =find(closest_point(2) == x_path(2,:));
cp_idx = union(cp_x_idx,cp_y_idx);
idx_dif = cp_idx - X_idx;

next_speed_sign_path= min(idx_dif((find( idx_dif >=0)))); % INDICE EM X_PATH em rela��o ao X
dist_int=0;

for i=X_idx+1:X_idx+next_speed_sign_path
    dist_int = dist_int+ norm(x_path(:,i).*[x_scale;y_scale] -x_path(:,i-1).* [x_scale;y_scale]);
end

if  dist_int > PEDESTRIAN_CROSSING_VISIBILITY %[METROS]
    return
end
%%% CROSSING V�LIDA - TEMPORALMENTE ANTES DO FIM
v = v/delta_t; % [m/s]
t_car_crossing = dist_int/v; %[s]

if   (sim_time + t_car_crossing > valid_crossings_times(1,1)-SAFETY_TIME_TRESH &&  sim_time + t_car_crossing < valid_crossings_times(1,1)+valid_crossings_times(2,1)+SAFETY_TIME_TRESH)
    %ROTA DE COLIS�O
    K = X_idx + (next_speed_sign_path-1);

    
elseif v < V_STOP_TRESH && sim_time < valid_crossings_times(1,1)+valid_crossings_times(2,1)+SAFETY_TIME_TRESH
    K = X_idx + (next_speed_sign_path-1);
end

end

