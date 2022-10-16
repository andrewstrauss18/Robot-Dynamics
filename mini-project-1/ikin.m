function q = ikin(S,M,currentQ,targetPose)
    A = [skew(targetPose(1:3)), targetPose(4:6); 0 0 0 0];
    T = expm(A);
    [R,p] = TransToRp(T);
    R = [0 0 -1; 0 1 0; 1 0 0]';                                                                                                                                               
    
    d1 = 0.352;
    a1 = 0.070;
    a2 = 0.360;
    d4 = 0.380;
    d6 = 0.065;

    Xc = p(1) + d6*R(1,2);
    Yc = p(2) + d6*R(2,2);
    Zc = p(3) + d6*R(3,2);

    q1 = atan2(Yc, Xc);

    r = sqrt(Xc^2+Yc^2);
    m = d1 - Zc;
    n = r - a1;
    alpha = atan2(m,n);
    l = sqrt(m^2+n^2);
    D = (a2^2+l^2-d4^2)/(2*a2*l);
    beta = atan2(sqrt(1-D^2),D);
    q2 = alpha+beta;

    E = (a2^2+d4^2-l^2)/(2*a2*d4);
    phi = atan2(sqrt(1-E^2),E);
    q3 = phi - pi/2;
    
%     q4 = atan2(cos(q1)*cos(q2+q3)*R(1,3) + sin(q1)*cos(q2+q3)*R(2,3) + sin(q2+q3)*R(3,3), -cos(q1)*sin(q2+q3)*R(1,3) - sin(q1)*sin(q2+q3)*R(2,3) + cos(q2+q3)*R(3,3));
%     q5 = atan2(sin(q1)*R(1,3) - cos(q1)*R(2,3), sqrt(1-(sin(q1)*R(1,3) - cos(q1)*R(2,3))^2));
%     q6 = atan2(-sin(q1)*R(1,1)+cos(q1)*R(2,1), sin(q1)*R(1,2)-cos(q1)*R(2,2));

    R13 = [  cos(q3)*(cos(q1)*cos(q2) + (8673110331987511*sin(q1)*sin(q2))/2361183241434822606848) - sin(q3)*(cos(q1)*sin(q2) - (8673110331987511*cos(q2)*sin(q1))/2361183241434822606848), (8673110331987511*cos(q3)*(cos(q1)*sin(q2) - (8673110331987511*cos(q2)*sin(q1))/2361183241434822606848))/2361183241434822606848 - (4503599627309731*sin(q1))/4503599627370496 + (8673110331987511*sin(q3)*(cos(q1)*cos(q2) + (8673110331987511*sin(q1)*sin(q2))/2361183241434822606848))/2361183241434822606848, (1084138791491125*sin(q1))/295147905179352825856 + (9007199254680227*cos(q3)*(cos(q1)*sin(q2) - (8673110331987511*cos(q2)*sin(q1))/2361183241434822606848))/9007199254740992 + (9007199254680227*sin(q3)*(cos(q1)*cos(q2) + (8673110331987511*sin(q1)*sin(q2))/2361183241434822606848))/9007199254740992;
            - cos(q3)*((8673110331987511*cos(q1)*sin(q2))/2361183241434822606848 - cos(q2)*sin(q1)) - sin(q3)*((8673110331987511*cos(q1)*cos(q2))/2361183241434822606848 + sin(q1)*sin(q2)), (4503599627309731*cos(q1))/4503599627370496 + (8673110331987511*cos(q3)*((8673110331987511*cos(q1)*cos(q2))/2361183241434822606848 + sin(q1)*sin(q2)))/2361183241434822606848 - (8673110331987511*sin(q3)*((8673110331987511*cos(q1)*sin(q2))/2361183241434822606848 - cos(q2)*sin(q1)))/2361183241434822606848, (9007199254680227*cos(q3)*((8673110331987511*cos(q1)*cos(q2))/2361183241434822606848 + sin(q1)*sin(q2)))/9007199254740992 - (1084138791491125*cos(q1))/295147905179352825856 - (9007199254680227*sin(q3)*((8673110331987511*cos(q1)*sin(q2))/2361183241434822606848 - cos(q2)*sin(q1)))/9007199254740992;
                                                                    - (9007199254680227*cos(q2)*sin(q3))/9007199254740992 - (9007199254680227*cos(q3)*sin(q2))/9007199254740992,                                                                                                                                                    (1084138791491125*cos(q2)*cos(q3))/295147905179352825856 - (1084138791491125*sin(q2)*sin(q3))/295147905179352825856 - 1084138791491125/295147905179352825856,                                                                                                                                                  (4503599627309731*cos(q2)*cos(q3))/4503599627370496 - (4503599627309731*sin(q2)*sin(q3))/4503599627370496 + 1043926651106815/77371252455336267181195264];

    R = R13'*R;

% E = cos(R(3,3));
% q5 = atan2(sqrt(1-E^2),E);
% F = sin(R(2,3)/sin(q5));
% G = sin(R(3,2)/sin(q5));
% q4 = atan2(F, sqrt(1-F^2));
% q6 = atan2(G, sqrt(1-G^2));

    q5 = acos(R(3,3));
    q4 = asin(R(2,3)/sin(q5));
    q6 = asin(R(3,2)/sin(q5));

% E = cos(R(3,3));
% q5 = atan2(sqrt(1-E^2),E);
% F = -cos(R(1,3)/sin(q5));
% G = cos(R(3,1)/sin(q5));
% q4 = atan2(sqrt(1-F^2),F);
% q6 = atan2(sqrt(1-G^2), G);

% q5 = acos(R(3,3));
% q4 = -acos(R(1,3)/sin(q5));
% q6 = acos(R(3,1)/sin(q5));

    q = [q1, q2, q3, q4, q5, q6];

%     o = fkine(S,M,q);
%     O = o(1:3,4);
%     scatter3(O(1),O(2),O(3))





%  T = fkine(S,M,currentQ);
%     currentPose = MatrixLog6(T);
%     currentPose = [currentPose(3,2) ...
%                    currentPose(1,3) ...
%                    currentPose(2,1) ...
%                    currentPose(1:3,4)']';
%                
%     Mthreshhold = 0.0824;
%     b = 0.107;  
%     while norm(targetPose - currentPose) > 1e-3
%         J = jacob0(S,currentQ);
%         Jt = J';
%         distToPose = targetPose - currentPose;
%         m = sqrt(det(J*Jt));
%         mSwitch = .01;
%         a = .55;
%         lambda = b*(1-(m/Mthreshhold));
%         switchSpot = .5;
% 
%         if norm(distToPose) < switchSpot || m < mSwitch
%             deltaQ = Jt*pinv(J*Jt+lambda^2*eye(6))*(distToPose);
%         else
%             deltaQ = a*Jt*(distToPose);
%         end
% 
%         currentQ = currentQ + deltaQ';
% 
%         T = fkine(S,M,currentQ);
%         currentPose = MatrixLog6(T);
%         currentPose = [currentPose(3,2) ...
%                        currentPose(1,3) ...
%                        currentPose(2,1) ...
%                        currentPose(1:3,4)']';
%     end
%     q = currentQ;
end