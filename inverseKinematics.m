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


%posicao A1 A2 A3   - Base_link ate wrist_link - Testado e a funcionar so
%dando uma solucao
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

teta1_2 = teta1 + pi
teta2_2 = -teta2
teta3_2 = teta3 + pi

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%falta testar - nao da para testar no simulador devido as restricoes das
%joints no braco

teta1_novo = teta1

%teta2_novo = 90 - (aux_phi_2 - aux_phi_1)
teta2_novo = (180 - (aux_phi_2 + aux_phi_1))*pi/180

teta3_novo = ((360 - (aux_phi_3 + 90))-aux_simplified)*pi/180

end