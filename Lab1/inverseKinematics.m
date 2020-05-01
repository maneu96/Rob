function [O1,O2,O3,O4,O5,O6] = inverseKinematics(A1,A2,A3,A4,A5,A6)

%Manipulator Dimensions
Base = 103;
Shoulder = 80;
Desvio_Arm = 15;
Arm = 210;
Elbow1 = 30;
Elbow2 = 41.5;
Forearm = 180;
Wrist = 23.7;
Hand = 5.5;

%%%Transformation base_link to tool_link
c1 = cos(A4);
c2 = cos(A5);
c3 = cos(A6);
s1 = sin(A4);
s2 = sin(A5);
s3 = sin(A6);

R_base_tool = [c1*c2*c3-s1*s3, -c3*s1-c1*c2*s3, c1*s2;
               c1*s3+c2*c3*s1, c1*c3-c2*s1*s3, s1*s2;
               -c3*s2, s2*s3, c2];
P_base_tool = [A1;A2;A3];

T_base_tool = [R_base_tool, P_base_tool;
               0, 0, 0, 1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Calcular a posicao da joint 5
P_base_wrist = T_base_tool*[-Wrist 0 0 1]';
P_base_wrist = P_base_wrist(1:3);

%posicao A1 A2 A3   - Base_link ate wrist_link - Testado e a funcionar so
%dando uma solucao
%Step 1: Inverse Kinematics for position in first 3 joints
X0_3 = P_base_wrist(1);
Y0_3 = P_base_wrist(2);
Z0_3 = P_base_wrist(3);

%%%%%%%%%%%% O mais provavel e isto estar mal %%%%%%%%%%%%%%%%%%%%%
teta1 = (atan2d(Y0_3, X0_3))*pi/180;

if(X0_3>=-30 && X0_3<0)
    teta1 = teta1-pi;
end

teta1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r1 = sqrt((X0_3^2)+(Y0_3^2));

r2 = Z0_3 - (Base+Shoulder);

r3 = sqrt((r1^2)+(r2^2));

d_aux = sqrt(((Forearm+Elbow2)^2)+(Elbow1^2));

aux_phi_1 = acosd(((d_aux^2)-(Arm^2)-(r3^2))/(-2*Arm*r3));

aux_phi_2 = atan2d(r2,r1);

aux_phi_3 = acosd(((r3^2)-(d_aux^2)-(Arm^2))/(-2*d_aux*Arm));

teta2 = -(90 - (aux_phi_1 + aux_phi_2))*pi/180


%%%%%%%%%%%%%%%%%%%%%%%% Verificar se isto esta certo %%%%%%%%%%%%%%%%%%
aux_simplified = -asind(Elbow1/d_aux);
if (X0_3<0)
    aux_simplified = -aux_simplified;
end

teta3 = (aux_phi_3 - 90 + aux_simplified)*pi/180

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% teta1_2 = teta1 + pi
% teta2_2 = -teta2
% teta3_2 = teta3 + pi

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%falta testar - nao da para testar no simulador devido as restricoes das
%joints no braco

% teta1_novo = teta1
% 
% %teta2_novo = 90 - (aux_phi_2 - aux_phi_1)
% teta2_novo = (180 - (aux_phi_2 + aux_phi_1))*pi/180
% 
% teta3_novo = ((360 - (aux_phi_3 + 90))-aux_simplified)*pi/180

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Step 2: Forward Kinematics on first 3 joints to get rotation part, R0_3
% T= @(alpha,a,d,theta) [cos(theta)              -sin(theta)              0           a;
%                        sin(theta)*cos(alpha)   cos(theta)*cos(alpha)    -sin(alpha) -sin(alpha)*d;
%                        sin(theta)*sin(alpha)   cos(theta)*sin(alpha)    cos(alpha)  cos(alpha)*d;
%                        0                       0                        0           1];
% 
% %assumindo teta4 = 0
% Table= [0   0   103  teta1;
%         0   0   80   0 ;
%     -pi/2   0   0    teta2;
%         0   0   0    -pi/2;
%         0  210  0    teta3;
%     -pi/2  30   0    0;
%         0  0   221.5  0;];
%     
% Transf= eye(4);    
% for i=1:size(Table,1)
%     Transf=Transf* T(Table(i,1),Table(i,2),Table(i,3),Table(i,4));
% end
% 
% R0_3 = Transf(1:3,1:3);


TBase_J1 = [1 0 0 0;
            0 1 0 0;
            0 0 1 Base;
            0 0 0 1];

TJ1_J2 = homo_matrix_andre(teta1,[0,0,Shoulder],"z");

aux_x = Arm * sind((teta2*180/pi));
aux_z = Arm * cosd((teta2*180/pi));
TJ2_J3 = homo_matrix_andre(teta2,[aux_x,0,aux_z],"y");

aux_x = Elbow2 * cosd((teta3*180/pi)) + Elbow1 * sind((teta3*180/pi));
aux_z = -Elbow2 * sind((teta3*180/pi)) + Elbow1 * cosd((teta3*180/pi));
TJ3_J4 = homo_matrix_andre(teta3,[aux_x,0,aux_z],"y");

TJ4_J5 = homo_matrix_andre(0,[Forearm,0,0],"x");

T0_3 = TBase_J1*TJ1_J2*TJ2_J3*TJ3_J4*TJ4_J5;

R0_3 = T0_3(1:3,1:3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Step 3: Find the inverse of the Ro_3 matrix

%R3_6 = inv(R0_3)*R_base_tool;
R3_6 = R0_3\R_base_tool;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Step 4: Forward Kinematics on the last 3 joints and pull out the rotation
%part, R3_6

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Step 5: Specify what you want the rotation matrix R0_6 to be

% ZYZ como esta no enunciado

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Step 6: Given a desired X,Y and Z position, solve for the first 3 joints
%using step 1

%%%%%%%%%%%%%%%%%%%% Saber qual a matrix de rotacao R3_6 %%%%%%%%%%%%%%%%%%
% syms A4 A5 A6
% 
% Table= [-pi/2  0   0    A4;
%         0  0   0  0;
%      pi/2  0    0    A5;
%     -pi/2  0  0 A6;];
% 
% Transf= eye(4);    
% for i=1:size(Table,1)
%     Transf=Transf* T(Table(i,1),Table(i,2),Table(i,3),Table(i,4));
% end
% 
% R3_6 = Transf(1:3,1:3)

% R3_6 =
%  
% [   cos(A4)*cos(A5)*cos(A6) - sin(A4)*sin(A6), - cos(A6)*sin(A4) - cos(A4)*cos(A5)*sin(A6), -cos(A4)*sin(A5)]
% [                             cos(A6)*sin(A5),                            -sin(A5)*sin(A6),          cos(A5)]
% [ - cos(A4)*sin(A6) - cos(A5)*cos(A6)*sin(A4),   cos(A5)*sin(A4)*sin(A6) - cos(A4)*cos(A6),  sin(A4)*sin(A5)]

% syms A4 A5 A6
% 
% aux_a4 = [1,0,0;
%           0,cos(A4),-sin(A4);
%           0,sin(A4),cos(A4)];
%       
% aux_a5 = [cos(A5),0,sin(A5);
%           0, 1, 0;
%           -sin(A5),0,cos(A5)];
% 
% aux_a6 = [1,0,0;
%           0,cos(A6),-sin(A6);
%           0,sin(A6),cos(A6)];
%       
% Aux = aux_a4*aux_a5*aux_a6
% 
% Aux =
%  
% [          cos(A5),                           sin(A5)*sin(A6),                             cos(A6)*sin(A5)]
% [  sin(A4)*sin(A5), cos(A4)*cos(A6) - cos(A5)*sin(A4)*sin(A6), - cos(A4)*sin(A6) - cos(A5)*cos(A6)*sin(A4)]
% [ -cos(A4)*sin(A5), cos(A6)*sin(A4) + cos(A4)*cos(A5)*sin(A6),   cos(A4)*cos(A5)*cos(A6) - sin(A4)*sin(A6)]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Step 7: Plug in those variables and use the rotation matrix to solve for
%the last 3 joints


%Esta aqui algo errado uma vez que R3_6 nao e igual a R3_6_teste
Teta5 = acos(R3_6(1,1))

if(Teta5>-1e-6 && Teta5<1e-6)
    Teta4 = 0
    Teta6 = acos(R3_6(3,3))
    
else
    %Teta6 = asin(R3_6(1,2)/(sin(Teta5)))
    Teta6 = atan2(R3_6(1,2),R3_6(1,3))
    %Teta4 = asin(R3_6(2,1)/(sin(Teta5)))
    Teta4 = atan2(-R3_6(2,1),R3_6(3,1))
end


Teste_R0_3 = [          cos(Teta5),                           sin(Teta5)*sin(Teta6),                             cos(Teta6)*sin(Teta5)
                sin(Teta4)*sin(Teta5), cos(Teta4)*cos(Teta6) - cos(Teta5)*sin(Teta4)*sin(Teta6), - cos(Teta4)*sin(Teta6) - cos(Teta5)*cos(Teta6)*sin(Teta4)
               -cos(Teta4)*sin(Teta5), cos(Teta6)*sin(Teta4) + cos(Teta4)*cos(Teta5)*sin(Teta6),   cos(Teta4)*cos(Teta5)*cos(Teta6) - sin(Teta4)*sin(Teta6)];

end