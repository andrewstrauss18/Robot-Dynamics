function T = twist2ht(S,theta)
    omega = S(1:3,:);
    v = S(4:6,:);
    skewOmega = skew(omega);
    R = axisangle2rot(omega, theta);
    P = (eye(3)*theta + (1 - cos(theta))*skewOmega + (theta - sin(theta))*skewOmega^2)*v;
    T = [R     P;
         0 0 0 1];
end
