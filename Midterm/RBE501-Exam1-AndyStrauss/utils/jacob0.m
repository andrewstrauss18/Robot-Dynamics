function J = jacob0(S,q)
    N = size(S,2);
    J = S(:,1);
    T = eye(4);
    for i = 2:N
        T = T*twist2ht(S(:,i-1),q(i-1));
        
        V = adjoint(S(:,i), T);
        
        J = [J V];
    end
end