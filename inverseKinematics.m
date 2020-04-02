function [O] = inverseKinematics(A1,A2,A3,A4,A5,A6)

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

%Max angle range
J1_min = -175*pi/180;
J1_max = 175*pi/180;
J2_min = -36.7*pi/180;
J2_max = 90*pi/180;
J3_min = -80*pi/180;
J3_max = 90*pi/180;
J4_min = -175*pi/180;
J4_max = 175*pi/180;
J5_min = -110*pi/180;
J5_max = 100*pi/180;
J6_min = -147.5*pi/180;
J6_max = 147.5*pi/180;

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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Calcular a posicao da joint 5
P_base_wrist = T_base_tool*[-Wrist 0 Hand 1]';
P_base_wrist = P_base_wrist(1:3);

%posicao A1 A2 A3   - Base_link ate wrist_link - Testado e a funcionar so
%dando uma solucao
%Step 1: Inverse Kinematicss for position in first 3 joints
X0_3 = P_base_wrist(1);
Y0_3 = P_base_wrist(2);
Z0_3 = P_base_wrist(3);

teta1 = (atan2d(Y0_3, X0_3))*pi/180;

teta1 = round(teta1*100)/100;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r1 = sqrt((X0_3^2)+(Y0_3^2));

r2 = Z0_3 - (Base+Shoulder);

r3 = sqrt((r1^2)+(r2^2));

d_aux = sqrt(((Forearm+Elbow2)^2)+(Elbow1^2));

aux_phi_1 = acosd(((d_aux^2)-(Arm^2)-(r3^2))/(-2*Arm*r3));

aux_phi_2 = atan2d(r2,r1);

aux_phi_3 = acosd(((r3^2)-(d_aux^2)-(Arm^2))/(-2*d_aux*Arm));

teta2 = -(90 - (aux_phi_1 + aux_phi_2))*pi/180;

teta2 = round(teta2*100)/100;

aux_simplified = -asind(Elbow1/d_aux);
if (round(X0_3)<0)
    aux_simplified = -aux_simplified;
end

teta3 = (aux_phi_3 - 90 + aux_simplified)*pi/180;

teta3 = round(teta3*100)/100;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Segunda opcao quando teta1 roda 180 graus %%%
teta1_2 = teta1 + pi;
teta1_2 = round(teta1_2*100)/100;

teta2_2 = -teta2;
teta2_2 = round(teta2_2*100)/100;

teta3_2 = pi - teta3;
teta3_2 = round(teta3_2*100)/100;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
teta1_novo = teta1;

teta2_novo = -(90 - (aux_phi_2 - aux_phi_1))*pi/180;
teta2_novo = round(teta2_novo*100)/100;

teta3_novo = ((360 - (aux_phi_3 + 90))+aux_simplified)*pi/180;
teta3_novo = round(teta3_novo*100)/100;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
teta1_novo_2 = teta1_novo + pi;
teta1_novo_2 = round(teta1_novo_2*100)/100;

teta2_novo_2 = -teta2_novo;
teta2_novo_2 = round(teta2_novo_2*100)/100;

teta3_novo_2 = pi - teta3_novo;
teta3_novo_2 = round(teta3_novo_2*100)/100;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Step 2: Forward Kinematics on first 3 joints to get rotation part, R0_3
TBase_J1 = [1 0 0 0;
            0 1 0 0;
            0 0 1 0;
            0 0 0 1];

TJ1_J2 = homo_matrix_andre(teta1,[0,0,0],"z");

TJ2_J3 = homo_matrix_andre(-teta2,[0,0,0],"y");

TJ3_J4 = homo_matrix_andre(-teta3,[0,0,0],"y");

T0_3 = TBase_J1*TJ1_J2*TJ2_J3*TJ3_J4;

R0_3 = T0_3(1:3,1:3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Step 3: Find the inverse of the Ro_3 matrix
R3_6 = R0_3\R_base_tool;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Step 4: Forward Kinematics on the last 3 joints and pull out the rotation
%part, R3_6

% R3_6 =
%  
% [          cos(A5),                           sin(A5)*sin(A6),                             cos(A6)*sin(A5)]
% [  sin(A4)*sin(A5), cos(A4)*cos(A6) - cos(A5)*sin(A4)*sin(A6), - cos(A4)*sin(A6) - cos(A5)*cos(A6)*sin(A4)]
% [ -cos(A4)*sin(A5), cos(A6)*sin(A4) + cos(A4)*cos(A5)*sin(A6),   cos(A4)*cos(A5)*cos(A6) - sin(A4)*sin(A6)]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Step 5: Specify what you want the rotation matrix R0_6 to be

% ZYZ como esta no enunciado

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Step 6: Given a desired X,Y and Z position, solve for the first 3 joints
%using step 1

%Done in step 1

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Step 7: Plug in those variables and use the rotation matrix to solve for
%the last 3 joints
Teta5 = acos(R3_6(1,1));
Teta5 = (round(Teta5*10)/10);

if(~isreal(Teta5))
    error("No solutions");
end

if(Teta5>-1e-6 && Teta5<1e-6)
    Teta4 = 0;
    Teta6 = atan2(R3_6(1,2),R3_6(1,3));
else 
    Teta6 = atan2(R3_6(1,2),R3_6(1,3))+pi;
    Teta4 = atan2(R3_6(2,1),-R3_6(3,1))+pi;
end

Teta5_2 = -Teta5;
if(Teta5_2>-1e-6 && Teta5_2<1e-6)
    Teta4_2 = 0;
    Teta6_2 = atan2(R3_6(1,2),R3_6(1,3));
else 
    Teta6_2 = atan2(R3_6(1,2),R3_6(1,3));
    Teta4_2 = atan2(R3_6(2,1),-R3_6(3,1));
end

Teta4 = (round(Teta4*100)/100);
Teta6 = (round(Teta6*100)/100);

Teta4_2 = (round(Teta4_2*100)/100);
Teta6_2 = (round(Teta6_2*100)/100);

O = [teta1,teta2,teta3,Teta4,Teta5,Teta6;
     teta1_2,teta2_2,teta3_2,Teta4,Teta5,Teta6;
     teta1_novo,teta2_novo,teta3_novo,Teta4,Teta5,Teta6;
     teta1_novo_2,teta2_novo_2,teta3_novo_2,Teta4,Teta5,Teta6;
     teta1,teta2,teta3,Teta4_2,Teta5_2,Teta6_2;
     teta1_2,teta2_2,teta3_2,Teta4_2,Teta5_2,Teta6_2;
     teta1_novo,teta2_novo,teta3_novo,Teta4_2,Teta5_2,Teta6_2;
     teta1_novo_2,teta2_novo_2,teta3_novo_2,Teta4_2,Teta5_2,Teta6_2];

O=unique(O,'rows','stable');

index_saved = [];
for i = 1:size(O,1)
    if(~isreal(O(i,:)))
        index_saved = [index_saved i];
    end
end

O(index_saved,:)=[];

for linha = 1:size(O,1)
    for col = 1:6
        if(O(linha,col)>pi)
            O(linha,col) = O(linha,col)-(2*pi);
        end
        if(O(linha,col)<-pi)
            O(linha,col) = O(linha,col)+(2*pi);
        end
    end
end

saved_index = [];
for l = 1: size(O,1)
    %joint limitations
    c_line = O(l,:);
    if(c_line(1)<J1_min || c_line(1)>J1_max || c_line(2)<J2_min || c_line(2)>J2_max || c_line(3)<J3_min || c_line(3)>J3_max || c_line(4)<J4_min || c_line(4)>J4_max ||...
            c_line(5)<J5_min || c_line(5)>J5_max || c_line(6)<J6_min || c_line(6)>J6_max)
        saved_index = [saved_index l];
    end
end

O(saved_index,:)=[];

end