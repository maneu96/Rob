function [phi_save,v_save, a_save, energy_spent, X, success] = robot_simulation(G, p_inicial, x_path, stop_signs,...
    speed_limit_signs, pedestrian_crossing_signs, pedestrian_crossing_pos, pedestrian_crossing_times, MAX_BATTERY)

X=x_path(:,1); % COORDENADAS DO CARRO

%DEFINES
STOP_SIGN_WAITING_TIME = 1; %[s]
M = 810; %[Kg]


%VARS
phi=0;
v_save=[0];
phi_save = 0;

%X_PATH
paths=stop_sign_handler(stop_signs, x_path);

% INIT 1º CICLO
x_path = cell2mat(paths(1)); %% 1º SUB PATH
[theta_path] = get_theta_path(cell2mat(paths(1)));
theta=theta_path(1);

P0 = 10; v=0;v_phi=0;wait_count=0;path_counter=1;
i=2;K=1;delta_t =0.1;
prev_speed_limit=0;
speed_limit=10; % [m/s] = 25,2 [km/h]
while 1%K<=length(x_path)
    
    if (norm(X(:,i-1)-x_path(:,K))<8) % DISTANCIA À REF 'k'
        if K==length(x_path) %ULTIMA REF DO SUBPATH
            if v/delta_t< 0.25% VELOCIDADE +-=0
                if length(paths)== path_counter %CHEGA AO DESTINO DO ULTIMO SUBPATH
                    break
                else %WAITS
                    wait_count = wait_count+1;
                    if wait_count >= STOP_SIGN_WAITING_TIME/delta_t %SELECIONA NOVO SUBPATH
                        path_counter = path_counter+1;
                        x_path = cell2mat(paths(path_counter));
                        [theta_path] = get_theta_path(x_path);
                        wait_count=0; K=1;
                    end
                end
            end
        else
            
            speed_limit_aux = speed_limit_handler(speed_limit_signs, X(:,end), x_path, speed_limit);
            speed_limit_ts = pedestrian_traffic_sign_handler(pedestrian_crossing_signs,X(:,end), x_path, speed_limit_aux, prev_speed_limit);
            
            if speed_limit_ts ~= speed_limit
                prev_speed_limit = speed_limit;
                speed_limit = speed_limit_ts;
            end
            K=K+1;
        end    
    end
    
    %CONTROLO
    i
    [phi,v] = controlador(X(:,i-1),theta, x_path(:,K), theta_path(K+1), phi, v,v_phi, delta_t, speed_limit, prev_speed_limit);
    %E4 - EVENT
    [K] = pedestrian_crossing_handler(i*delta_t, pedestrian_crossing_pos, pedestrian_crossing_times, X(:,end), x_path, v, delta_t, K);
    %MODELO DO CARRO
    [X(:,i),theta]=robot(X(:,i-1),theta,phi,v, delta_t);
    
    %DEBUG
    phi_save = [phi_save, phi];
    v_save = [v_save , v];
    a_save(i) = (v - v_save(end-1)) / delta_t;
    v_phi = phi-phi_save(end-1);
    
    if a_save(i) <0 %ENERGIA
        energy_spent(i) = (P0)* abs(v) * delta_t;
    else
        energy_spent(i) = (M*a_save(i) + P0)* abs(v) * delta_t;
    end
    if sum(energy_spent)>= MAX_BATTERY
        success =0;
        return
    end
    i=i+1;
end
success =1;
end
