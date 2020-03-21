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

R_base_tool = [c1*c2*c3-s1*s3, -c3*s1-c1*c2*c3, c1*s2;
               c1*s3+c2*c3*s1, c1*c3-c2*s1*s3, s1*s2;
               -c3*s2, s2*s3, c2];
P_base_tool = [A1;A2;A3];

T_base_tool = [R_base_tool, P_base_tool;
               0, 0, 0, 1];

%posicao A1 A2 A3   - Base_link ate wrist_link - Testado e a funcionar so
%dando uma solucao
%Step 1: Inverse Kinematics for position in first 3 joints
X0_3 = A1;
Y0_3 = A2;
Z0_3 = A3;

teta1 = (atan2d(Y0_3, X0_3))*pi/180

r1 = sqrt((X0_3^2)+(Y0_3^2));

r2 = Z0_3 - (Base+Shoulder);

r3 = sqrt((r1^2)+(r2^2));

d_aux = sqrt(((Forearm+Elbow2)^2)+(Elbow1^2));

aux_phi_1 = acosd(((d_aux^2)-(Arm^2)-(r3^2))/(-2*Arm*r3));

aux_phi_2 = atan2d(r2,r1);

aux_phi_3 = acosd(((r3^2)-(d_aux^2)-(Arm^2))/(-2*d_aux*Arm));

teta2 = -(90 - (aux_phi_1 + aux_phi_2))*pi/180

aux_simplified = -asind(Elbow1/d_aux);
if (X0_3<0)
    aux_simplified = -aux_simplified;
end

teta3 = (aux_phi_3 - 90 + aux_simplified)*pi/180

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% teta1_2 = teta1 + pi
% teta2_2 = -teta2
% teta3_2 = teta3 + pi

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
T= @(alpha,a,d,theta) [cos(theta)              -sin(theta)              0           a;
                       sin(theta)*cos(alpha)   cos(theta)*cos(alpha)    -sin(alpha) -sin(alpha)*d;
                       sin(theta)*sin(alpha)   cos(theta)*sin(alpha)    cos(alpha)  cos(alpha)*d;
                       0                       0                        0           1];

%assumindo teta4 = 0
Table= [0   0   103  teta1;
        0   0   80   0 ;
    -pi/2   0   0    teta2;
        0   0   0    -pi/2;
        0  210  0    teta3;
    -pi/2  30   0    0;
        0  0   221.5  0;];
    
Transf= eye(4);    
for i=1:size(Table,1)
    Transf=Transf* T(Table(i,1),Table(i,2),Table(i,3),Table(i,4));
end

R0_3 = Transf(1:3,1:3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Step 3: Find the inverse of the Ro_3 matrix

%R3_6 = inv(R0_3)*R_base_tool;
R3_6 = R0_3\R_base_tool;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Step 4: Forward Kinematics on the last 3 joints and pull out the rotation
%part, R3_6

%Igual a do video

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




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Step 7: Plug in those variables and use the rotation matrix to solve for
%the last 3 joints


%Esta aqui algo errado uma vez que R3_6 nao e igual a R3_6_teste
Teta5 = acos(R3_6(2,3))

Teta6 = asin(R3_6(2,2)/(-sin(Teta5)))

Teta4 = acos(R3_6(1,3)/(-sin(Teta5)))



R3_6_teste = [cos(Teta4)*cos(Teta5)*cos(Teta6)-sin(Teta4)*sin(Teta6), - cos(Teta6)*sin(Teta4)-cos(Teta4)*cos(Teta5)*sin(Teta6), -cos(Teta4)*sin(Teta5);
                             cos(Teta6)*sin(Teta5),                            -sin(Teta5)*sin(Teta6),          cos(Teta5);
 - cos(Teta4)*sin(Teta6) - cos(Teta5)*cos(Teta6)*sin(Teta4),   cos(Teta5)*sin(Teta4)*sin(Teta6)-cos(Teta4)*cos(Teta6),  sin(Teta4)*sin(Teta5)];




end