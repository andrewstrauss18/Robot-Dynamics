function T = fkine(S,M,q)
    N = size(S,2);
    T = eye(4);
    for i = 1:N
        T = T*twist2ht(S(:,i),q(i));
    end
    T = T*M;
end