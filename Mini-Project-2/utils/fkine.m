function T = fkine(S,M,q,frame)
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