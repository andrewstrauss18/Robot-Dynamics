function R = axisangle2rot(omega,theta)
    skewOmega = skew(omega);
    R = eye(3) + sin(theta)*skewOmega + (1 - cos(theta))*skewOmega^2;
end
