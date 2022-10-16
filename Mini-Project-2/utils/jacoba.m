function J_a = jacoba(S,M,q)    
    T = fkine(S,M,q,'space');
    p = T(1: 3, 4);
    p_skew = skew(p);
    J_s = jacob0(S,q);
    J_vs = J_s(4:6,:);
    J_ws = J_s(1:3,:);
    
    J_a = J_vs - p_skew*J_ws;
end