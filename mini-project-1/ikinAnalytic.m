function q = ikinAnalytic(S_space,M,currentQ,targetPose)
              
    Mthreshhold = 0.0824;
    b = 0.107;  
    distToPose = norm(targetPose - currentPose);
    while distToPose > 1e-3
        pDistToPose = targetPose - currentPose;
        J_a = jacoba1(S_space,M,currentQ);
        J_at = J_a';
        m = sqrt(det(J_a*J_at));
        a = .55;
        lambda = b*(1-(m/Mthreshhold));
        switchSpot = .75;
        mSwitch = .01;

        if distToPose < switchSpot || m < mSwitch
            deltaQ = J_at*pinv(J_a*J_at+lambda^2*eye(3))*(pDistToPose);
        else
            deltaQ = a*J_at*(pDistToPose);
        end

        currentQ = currentQ + deltaQ';

        T = fkine1(S_body, M, currentQ, 'body');
        currentPose = T(1:3,4);
        
        distToPose = norm(targetPose - currentPose);
    end
    
    q = currentQ;
end

function V_b = twistspace2body1(V_s,T)
    V_b = adjoint(V_s, pinv(T));
end

function T = fkine1(S,M,q,frame)
    if strcmp(frame, "space") == 1
        N = size(S,2);
        T = eye(4);
        for i = 1:N
            T = T*twist2ht(S(:,i),q(i));
        end
        T = T*M;  
    elseif strcmp(frame, "body") == 1
        N = size(S,2);
        T = eye(4);
        for i = 1:N
            T = T*twist2ht(S(:,i),q(i));
        end
        T = M*T;
    else
        disp("invalid frame, try 'space' or 'body'")
        T = [];
    end
end

function J_a = jacoba1(S,M,q)    
    T = fkine(S,M,q,'space');
    p = T(1: 3, 4);
    p_skew = skew(p);
    J_s = jacob0(S,q);
    J_vs = J_s(4:6,:);
    J_ws = J_s(1:3,:);
    
    J_a = J_vs - p_skew*J_ws;
end
