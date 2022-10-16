function q = ikine2(S,M,currentQ,targetPose)
    A = [skew(targetPose(1:3)), targetPose(4:6); 0 0 0 0];
    T = expm(A);
    [R,p] = TransToRp(T);
    
    d1 = 0.352;
    a1 = 0.070;
    a2 = 0.360;
    d4 = 0.380;
    d6 = 0.065;

    xtip = p(1) - d6*R(1,2);
    ytip = p(2) - d6*R(2,2);
    ztip = p(3) - d6*R(3,2);

    %Solve for q1
    q1 = atan2(ytip, xtip);
    
    %Solve for q3
    s = (ztip - d1);
    r = sqrt((xtip - a1*cos(q1))^2 +(ytip - a1*sin(q1))^2);
    czeta = (r^2 + s^2 - (a2)^2 - (d4 + d6)^2)/(2*a2*(d4 + d6));

    szeta = sqrt(1-(czeta)^2);
    zeta= atan2(szeta,czeta);
    
    q3 = -(pi/2 + zeta);
    
    %Solve for q2
    omega = atan2(s,r);
    lambda = atan2((d4+d6)*sin(zeta),a2+(d4+d6)*cos(zeta));
    %q2 = -((omega - lambda) - pi/2);
    q2 = omega - lambda;
     
%     alpha = q4;
%     beta = q5;
%     gama = q6;
%     
%     R60 = [cos(alpha)*cos(beta), (cos(alpha)*sin(beta)*sin(gama))- sin(alpha)*cos(gama), (cos(alpha)*sin(beta)*cos(gama)) + sin(alpha)*sin(gama) ;
%            sin(alpha)*cos(beta), (sin(alpha)*sin(beta)*sin(gama)) + cos(alpha)*cos(gama), (sin(alpha)*sin(beta)*cos(gama)) - cos(alpha)*sin(gama) ;
%            -sin(beta), cos(beta)*sin(gama), cos(beta)*sin(gama)];
    
    R30 = [cos(q1)*cos(q2+q3), -cos(q1)*sin(q2+q3), sin(q1);
           sin(q1)*cos(q2+q3), -sin(q1)*sin(q2+q3), cos(q1);
           -sin(q2+q3), -cos(q2+q3), 0 ];

    RT30= transpose(R30);
    R63 = RT30*R;
    g11 = R63 (1,1);
    g12 = R63 (1,2);
    g23 = R63 (2,3);
    g31 = R63 (3,1);
    g32 = R63 (3,2);
    g33 = R63 (3,3);

    %Solve for q2
    q5 = atan2(sqrt((g31)^2 + (g32)^2), g33);

    %Solve for q2
    q4 = atan2(g32/sin(q5), - g31/sin(q5));
    
    %Solve for q2
    q6 = atan2(g23/sin(q5), g31/sin(q5));
    
    q = [q1 q2 q3 q4 q5 q6];
    
end

