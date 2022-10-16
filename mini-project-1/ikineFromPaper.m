% THIS PROGRAM IS USED TO SOLVE THE INVERSE KINEMATIC OF THE ABB IRB 140
% DEFINE A NON RETURN FUNCTION TO COMBINE ALL THE INVERSE FUNCTIONS TOGETHER IN ONE SCRIPT
function [NONRETURNFUNCTION] = ikineFromPaper( )
%INTERNATIONAL JOURNAL OF ELECTRONICS, MECHANICAL AND MECHATRONICS ENGINEERING Vol.7 Num.2 - 2017 (1383-1401) 1397
%Mohammed Almaged
% DECLERATION OF THE ROBOT PARAMETER
d1 = 352;
a1 = 70;
a2 = 360;
d4 = 380;
NOSOLUTION=1000;

% THIS PROGRAM IS DESIGNED TO SOLVE THE INVERSE WITH RESPECT TO Porg6 OR TCP
%ACCORDING TO USER SELECTION
sel = input ('TO SOLVE THE INVERSE WITH RESPECT TO FRAME6 PRESS 1 WHILE, TO SOLVE THEINVERSE WITH RESPECT TO TCP ENTER 2: ');
if (sel == 1)
d6 = 0;
elseif (sel == 2)
d6 = 65;
else
d6= 65;
end

% USER INTERFACE
xtip = input ('PLEASE ENTER THE GOAL POSTION X = ');
ytip = input ('PLEASE ENTER THE GOAL POSTION y = ');
ztip = input ('PLEASE ENTER THE GOAL POSTION z = ');
alpha= input ('PLEASE ENTER THE VALUE OF alpha IN DEGREE = ');
beta = input ('PLEASE ENTER THE VALUE OF beta IN DEGREE = ');
gama = input ('PLEASE ENTER THE VALUE OF gama IN DEGREE = ');

% CALCULATING ALL THE POSSIBLE VALUES FOR THETA1
theta1= atan2 (ytip,xtip);
theta11= pi + theta1;
THETA1 = theta1 * 180/pi;
THETA11= theta11 * 180/pi;

% CALCULATING ALL THE POSSIBLE VALUES FOR THETA3
s = (ztip - d1);
r = sqrt((xtip - a1*cos (theta1))^2 +(ytip - a1*sin(theta1))^2);
czeta = (r^2 + s^2 - (a2)^2 - (d4 + d6)^2)/(2 * a2 *(d4 + d6));

% SINGULARTIY CONDTION, CHECK IF THE POSTION WITHIN THE WORKSPACE OR NOT
if (abs(czeta) <= 1)
szeta = sqrt(1-(czeta)^2);
szeta1 = -szeta;
zeta= atan2(szeta,czeta);
zeta1= atan2(szeta1,czeta);
theta3 = -(pi/2 + zeta);
theta33 = -(pi/2 + zeta1);
THETA3 = conversion( theta3,50,-230);
THETA33 = conversion( theta33,50,-230);
else
theta3 = NOSOLUTION;


Forward and Inverse Kinematic Analysis and Validation of the ABB IRB 140 Industrial Robot
theta33= NOSOLUTION;
THETA3 = ('GOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR THETA3');
THETA33 = ('GOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR THETA33');
end
s = (ztip - d1);
r = sqrt((xtip - a1*cos (theta11))^2 +(ytip - a1*sin(theta11))^2);
czetai = (r^2 + s^2 - (a2)^2 - (d4 + d6)^2)/(2 * a2 *(d4 + d6));

% SINGULARTIY CONDTION, CHECK IF THE POSTION WITHIN THE WORKSPACE OR NOT
if (abs(czetai) <= 1)
szetai = sqrt(1-(czetai)^2);
szeta1i = -szetai;
zetai= atan2(szetai,czetai);
zeta1i= atan2(szeta1i,czetai);
theta3i = -(pi/2 + zetai);
theta33i = -(pi/2 + zeta1i);
THETA3i = conversion( theta3i,50,-230);
THETA33i = conversion( theta33i,50,-230);
else
theta3i=NOSOLUTION;
theta33i=NOSOLUTION;
THETA3i = ('GOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR THETA3i');
THETA33i = ('GOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR THETA33i');
end

% CALCULATING ALL THE POSSIBLE VALUES FOR THETA2

if (theta3 == NOSOLUTION)
THETA2 = ('GOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR THETA2');
THETA22 = ('GOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR THETA22');
else
theta2 = THE2(xtip,ytip,ztip,theta1,zeta);
theta22 = THE2COMP(xtip,ytip,ztip,theta1,zeta);
THETA2 = conversion( theta2,110,-90);
THETA22 = conversion( theta22,110,-90);
end

if (theta33 == NOSOLUTION)
THETA2i = ('GOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR THETA2i');
THETA22i = ('GOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR THETA22i');
else
theta2i = THE2(xtip,ytip,ztip,theta1,zeta1);
theta22i = THE2COMP(xtip,ytip,ztip,theta1,zeta1);
THETA2i = conversion( theta2i,110,-90);
THETA22i = conversion( theta22i,110,-90);
end

if (theta3i == NOSOLUTION)
THETA2j = ('GOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR THETA2j');
THETA22j = ('GOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR THETA22j');
else
theta2j = THE2(xtip,ytip,ztip,theta11,zetai);
INTERNATIONAL JOURNAL OF ELECTRONICS, MECHANICAL AND MECHATRONICS ENGINEERING Vol.7 Num.2 - 2017 (1383-1401) 1399
Mohammed Almaged
theta22j = THE2COMP(xtip,ytip,ztip,theta11,zetai);
THETA2j = conversion( theta2j,100,-90);
THETA22j = conversion( theta22j,100,-90);
end

if (theta33i == NOSOLUTION)
THETA2k = ('GOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR THETA2k');
THETA22k = ('GOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR THETA22k');
else
theta2k = THE2(xtip,ytip,ztip,theta11,zeta1i);
theta22k = THE2COMP(xtip,ytip,ztip,theta11,zeta1i);
THETA2k = conversion( theta2k,110,-90);
THETA22k = conversion( theta22k,110,-90);
end

% DISPLAY ALL THE POSSIBLE EIGHT SOLUTIONS, NOTE THAT EVERY TWO SOLUTIONS FORM
 ONLY ONE SOLUTION SET
disp ( ' THETA 1,2,3 SOLUTIONS')
disp ( ' SET 1')
SOL1 =[ THETA1, THETA2, THETA3]
SOL2 =[ THETA1, THETA22, THETA3]
disp ( ' SET 2')
SOL3 =[ THETA1, THETA2i, THETA33]
SOL4 =[ THETA1, THETA22i, THETA33]
disp ( ' SET 3')
SOL5 =[ THETA11, THETA2j, THETA3i]
SOL6 =[ THETA11, THETA22j, THETA3i]
disp ( ' SET 4')
SOL7 =[ THETA11, THETA2k, THETA33i]
SOL8 =[ THETA11, THETA22k, THETA33i]

% SOLVING THE SECOND KINEMATIC SUB-PROBLEM (ORIENTATION)
alpha = alpha * pi/180;
beta = beta * pi/180;
gama = gama * pi/180;
R60 = [cos(alpha).*cos(beta), (cos(alpha).*sin(beta).*sin(gama))- sin(alpha).*cos(gama),
 (cos(alpha).*sin(beta).*cos(gama)) + sin(alpha).*sin(gama) ;
 sin(alpha).*cos(beta), (sin(alpha).*sin(beta).*sin(gama)) + cos(alpha).*cos(gama),
 (sin(alpha).*sin(beta).*cos(gama)) - cos(alpha).*sin(gama) ;
 - sin (beta), cos (beta).*sin (gama), cos (beta).*sin (gama)];
R30 = [cos(theta1).*cos(theta2+theta3), -cos(theta1).*sin(theta2+theta3), sin(theta1);
 sin (theta1).*cos(theta2+theta3), -sin(theta1).*sin(theta2+theta3), cos(theta1);
 -sin(theta2+theta3), -cos(theta2+theta3), 0 ];


%Forward and Inverse Kinematic Analysis and Validation of the ABB IRB 140 Industrial Robot
RT30= transpose (R30);
R63 = RT30 * R60 ;
g11 = R63 (1,1);
g12 = R63 (1,2);
g23 = R63 (2,3);
g31 = R63 (3,1);
g32 = R63 (3,2);
g33 = R63 (3,3);

% THETA 4,5,6 CALCULATION

theta5 = atan2 ( sqrt((g31)^2 +(g32)^2), g33);
if(theta5 == 0)
THETA4= 0
THETA5= 0
theta6 = atan2 (-g12, g11);
THETA6= theta6*180/pi
elseif (theta5 == pi)
THETA4= 0
THETA5= 0
theta6 = atan2 (g12,-g11);
THETA6= theta6*180/pi
else
theta4 = atan2 (g32/ sin (theta5), - g31/ sin (theta5));
theta6 = atan2 (g23/ sin (theta5), g31/ sin (theta5));
THETA4= conversion( theta4,200,-200);
THETA5= conversion( theta5,115,-115);
THETA6= conversion( theta6,400,-400);

% FLIPPED POSTION

theta44 = theta4 + pi;
theta55 = -theta5;
theta66 = theta6+pi;
THETA44= conversion( theta44,200,-200);
THETA55= conversion( theta55,115,-115);
THETA66= conversion( theta66,400,-400);
disp ( ' THETA 4,5,6 SOLUTIONS')
Solution1 = [THETA4,THETA5,THETA6]
Solution2 = [THETA44,THETA55,THETA66]
end

% FIRST POSSIBLE SOLUTION OF THETA2 FUNCTION

function RES = THE2(xtip,ytip,ztip,theta1,zeta)
 s = (ztip - d1);
 r = sqrt((xtip - a1*cos (theta1))^2 +(ytip - a1*sin(theta1))^2);
omega = atan2 (s, r);
lenda = atan2 (( d4+d6) * sin (zeta) , a2+( d4+d6)* cos (zeta));
RES = - ((omega - lenda)- ( pi/2)) ;
end
% INTERNATIONAL JOURNAL OF ELECTRONICS, MECHANICAL AND MECHATRONICS ENGINEERING Vol.7 Num.2 - 2017 (1383-1401) 1401
% Mohammed Almaged
% SECOND POSSIBLE SOLUTION OF THETA2 FUNCTION

function RES1 = THE2COMP(xtip,ytip,ztip,theta1,zeta)
s = (ztip - d1);
r = - sqrt((xtip - a1*cos (theta1))^2 +(ytip - a1*sin(theta1))^2);
omega = atan2 (s, r);
lenda = atan2 (( d4+d6) * sin (zeta) , a2+( d4+d6)* cos (zeta));
RES1 = - ((omega - lenda) - ( pi/2));
end

% JOINT ANGLES LIMIT FUNCTION

function OUT = conversion( theta,upperlimit,lowerlimit)
upperlimit = upperlimit * pi / 180;
lowerlimit = lowerlimit * pi / 180;
if (theta > upperlimit)
OUT = (' THE SOLUTION OUT OF JOINT ANGLE LIMIT ');
elseif (theta < lowerlimit)
OUT = (' THE SOLUTION OUT OF JOINT ANGLE LIMIT ');
else
OUT = theta * 180 / pi;
end

end

end