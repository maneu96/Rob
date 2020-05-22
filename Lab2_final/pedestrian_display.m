function  [] = pedestrian_display(i,delta_t, pedestrian_crossing_pos, pedestrian_crossing_time, app)

if isempty(pedestrian_crossing_pos)
    return
end
for j=1:size(pedestrian_crossing_pos, 2)
    if pedestrian_crossing_time(1,j) <= i*delta_t && ...
        (pedestrian_crossing_time(1,j) + pedestrian_crossing_time(2,j)) >= i*delta_t
        c(j,:) = [1 0 0];
    else
        c(j,:) = [0 1 0];
    end
end
    app.pedestrian_crossing_pos_scatter = scatter(app.UIAxes, pedestrian_crossing_pos(1,:),pedestrian_crossing_pos(2,:),...
        30, c, 'filled', 'd');
end

