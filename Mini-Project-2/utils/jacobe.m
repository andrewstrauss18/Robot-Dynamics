function J_b = jacobe(S,M,q)    
    J_s = jacob0(S,q);
    T = fkine(S,M,q,'space');
    J_b = adjoint(J_s, pinv(T));
end
