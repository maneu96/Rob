function [matrix] = homo_matrix(angle,translation,axis) 

matrix = zeros(3,4);
angle_deg = angle*180/pi;

if axis=="x"
    matrix = [1 0 0 translation(1);
        0 cosd(angle_deg) -sind(angle_deg) translation(2);
        0 sind(angle_deg) cosd(angle_deg) translation(3);
        0 0 0 1];
    
elseif axis=="y"
    matrix = [cosd(angle_deg) 0 sind(angle_deg) translation(1);
        0 1 0 translation(2);
        -sind(angle_deg) 0 cosd(angle_deg) translation(3);
        0 0 0 1];
    
elseif axis=="z"
    matrix = [cosd(angle_deg) -sind(angle_deg) 0 translation(1);
        sind(angle_deg) cosd(angle_deg) 0 translation(2);
        0 0 1 translation(3);
        0 0 0 1];
    
else
    error("homo_matrix: Not a valid input");
end

end

