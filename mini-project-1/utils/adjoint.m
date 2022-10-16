function Vtrans = adjoint(V,T)
    R = T(1:3,1:3);
    p = T(1:3, 4);
    skewP = [0 -p(3) p(2);
             p(3) 0 -p(1);
             -p(2) p(1) 0];
    I = zeros(3);
    Adt = [R       I;
           skewP*R R];
    Vtrans = Adt*V;
end