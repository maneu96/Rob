function [ O ] = direct_kinematics( A1,A2,A3,A4,A5,A6)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% R_x=@(alpha) [1  0      0;
%           0 cos(alpha) -sin(alpha);
%           0 sin(alpha) cos(alpha)];
% R_z=@(theta) [cos(theta)  -sin(theta)   0;
%               sin(theta)   cos(a)       0;
%               1            0            0];
% T_x= @(a) [a;0;0];
% T_z= @(d) [0;0;d];

T= @(alpha,a,d,theta) [cos(theta)              -sin(theta)              0           a;
                       sin(theta)*cos(alpha)   cos(theta)*cos(alpha)    -sin(alpha) -sin(alpha)*d;
                       sin(theta)*sin(alpha)   cos(theta)*sin(alpha)    cos(alpha)  cos(alpha)*d;
                       0                       0                        0           1];

Table= [0   0   103  A1;
        0   0   80   0 ;
    -pi/2   0   0    A2;
        0   0   0    -pi/2;
        0  210  0    A3;
    -pi/2  30   0    A4;
        0  0   221.5  0;
     pi/2  0    0    A5;
    -pi/2  -5.5  23.7 A6
        0  0    0    pi/2;
     pi/2  0    0     0;
        0  0    0    pi/2];

Transf= eye(4);    
for i=1:size(Table,1)
    Transf=Transf* T(Table(i,1),Table(i,2),Table(i,3),Table(i,4));
end
%% Euler angles
R = Transf(1:3,1:3);

if(round(det(R))~=1)
    error('Matriz de rotacao errada');
end

if(R(3,3)<1)
    if(R(3,3)>-1)
        alpha = atan2(R(2,3),R(1,3));
        beta = acos(R(3,3));
        gama = atan2(R(3,2),-R(3,1));
    else    %R(3,3)=-1
        %Not a unique solution: gama ? alpha = atan2(R(2,1) ,R(2,2))
        alpha = -atan2(R(2,1),R(2,2));
        beta = pi;
        gama = 0;
    end
else    %R(3,3)=+1
    %Not a unique solution: gama + alpha = atan2(R(2,1) ,R(2,2))
    alpha = atan2(R(2,1),R(2,2));
    beta = 0;
    gama = 0; 
end
O(4:6)=[alpha;beta;gama];
O(1:3)=Transf(1:3,4);
end

