close all;
clear all;

p_inicial = [700;900]; %USER INPUT %%%%%%%%%
p_inicial = [340;900]; %USER INPUT %%%%%%%%%
p_final = [760; 1560]; %USER INPUT %%%%%%%%%
X= p_inicial; % COORDENADAS DO CARRO

%DEFINES
STOP_SIGN_WAITING_TIME = 1; %[s]
M = 810; %[Kg]

load G

%VARS
phi=0;
v_save=[0];
phi_save = 0;

stop_signs = [360 1360 624;1120 1000 1200] ; %STOP SIGNS POSITIONS
%stop_signs = [];

speed_limit_pos = [360 1360 624;1120 1000 1200] ; %SPEED LIMIT SIGNS POSITIONS
speed_limit_vel = [5.5 2.77 6.5]; % [m/s]
speed_limit_signs = [speed_limit_pos; speed_limit_vel];
%speed_limit_signs=[];

pedestrian_crossing_signs = [360 1360 624;1120 1000 1200] ; % PEDESTRIAN CROSSING SIGNS POSITIONS
pedestrian_crossing_times = [45 10 14; 20 5 4 ]; % [INICIO DA PASSAGEM; DURA��O]

% CODE %%%
[xq, vq] = get_path(p_inicial, p_final, G); %Gets all reference points in the path
x_path = [xq; vq];

paths=stop_sign_handler(stop_signs, x_path);

% INIT 1� CICLO
x_path = cell2mat(paths(1)); %% 1� SUB PATH
[theta_path] = get_theta_path(cell2mat(paths(1)));
theta=theta_path(1);

P0 = 1; v=0;v_phi=0;wait_count=0;path_counter=1;
i=2;K=1;delta_t =0.1; 
prev_speed_limit=0;
speed_limit=10; % [m/s] = 25,2 [km/h]
while 1%K<=length(x_path)

     if (norm(X(:,i-1)-x_path(:,K))<8) % DISTANCIA � REF 'k'
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
            if speed_limit_aux ~= speed_limit
                prev_speed_limit = speed_limit;
                speed_limit = speed_limit_aux;
            end
            K=K+1
        end
     end
    %CONTROLO
    [phi,v] = controlador(X(:,i-1),theta, x_path(:,K), theta_path(K+1), phi, v,v_phi, delta_t, speed_limit, prev_speed_limit);
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
   
    i=i+1;
end

figure;
imshow(imread('ist_map_detail.png')); hold on
scatter(x_path(1,:),x_path(2,:));
hold on;
plot(X(1,:),X(2,:));

figure()
plot( (1:length(a_save))*delta_t, a_save/delta_t);
hold on
plot( (1:length(a_save))*delta_t, v_save/delta_t);

figure()
plot( (1:length(phi_save))*delta_t, phi_save);

