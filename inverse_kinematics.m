function [ O1,O2,O3,O4,O5,O6 ] = inverse_kinematics (A1,A2,A3,A4,A5,A6)
%INVERSE_KINEMATICS Summary of this function goes here
%   Detailed explanation goes here
%%%Transformation base_link to tool_link
c1 = cos(A4);
c2 = cos(A5);
c3 = cos(A6);
s1 = sin(A4);
s2 = sin(A5);
s3 = sin(A6);

% R_base_tool = [c1*c2*c3-s1*s3, -c3*s1-c1*c2*s3, c1*s2;
%                c1*s3+c2*c3*s1, c1*c3-c2*s1*s3, s1*s2;
%                -c3*s2, s2*s3, c2];
R_base_tool = [c2, s2*s3, c3*s2;
               s1*s2, c1*c3-c2*s1*s3, -c1*s3-c2*c3*s1;
               -c1*s2, c3*s1+c1*c2*s3, c1*c2*c3-s1*s3];

P_base_tool = [A1;A2;A3];

T_base_tool = [R_base_tool, P_base_tool;
               0, 0, 0, 1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Calcular a posicao da joint 5
P_base_wrist = (T_base_tool)*[-23.7 0 5.5 1]';
P_base_wrist = P_base_wrist(1:3);

%posicao A1 A2 A3   - Base_link ate wrist_link - Testado e a funcionar so
%dando uma solucao
%Step 1: Inverse Kinematics for position in first 3 joints
X0_3 = P_base_wrist(1);
Y0_3 = P_base_wrist(2);
Z0_3 = P_base_wrist(3);

%%%%%%%%%%%% O mais provavel e isto estar mal %%%%%%%%%%%%%%%%%%%%%
teta1 = (atan2d(Y0_3, X0_3))*pi/180;

% if(X0_3>=-30 && X0_3<0)
%     teta1 = teta1-pi;
% end
% if(sqrt(X0_3^2 + Y0_3^2) < 210+sqrt(41.5^2+30^2))
%     acos(sqrt(X0_3^2 + Y0_3^2)/210+
% else
    c_teta2=(210^2 + (X0_3)^2 + Y0_3^2 - (41.5^2+30^2))/(2*210*sqrt(X0_3^2 + Y0_3^2));
    teta2 = acos(c_teta2);
    teta3 = asin((sqrt(X0_3^2 + Y0_3^2)-c_teta2*210)/(41.5^2+30^2));
    
end

