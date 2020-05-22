function [ACCEL_TRESH] = get_ACCEL(v,last_v, MAX_ACCEL_TRESH, delta_t)
tau= 0.3;

func_A= @(V) MAX_ACCEL_TRESH*exp(-tau*V);
v_aux = last_v/delta_t; %[m/s]

ACCEL_TRESH=func_A(v_aux);

if v<last_v %significa que o robot quer travar
    ACCEL_TRESH=MAX_ACCEL_TRESH;
elseif v>last_v %significa que o robot quer acelarar
    if ACCEL_TRESH> MAX_ACCEL_TRESH
        ACCEL_TRESH=MAX_ACCEL_TRESH;
    end
end

end

