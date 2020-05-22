function [ACCEL_TRESH] = accel_non_linear_block(ACCEL_TRESH, MAX_ACCEL_TRESH)

K = MAX_ACCEL_TRESH/ (log10(101));
start = 0.2;
xq =start:(MAX_ACCEL_TRESH-start)/100:MAX_ACCEL_TRESH;
start2=1.3;
vq = K*log10(start2:(100-start2)/100:101-(100-start2)/100);

map_idx = find(xq > ACCEL_TRESH );
ACCEL_TRESH = vq(map_idx(1));

end

