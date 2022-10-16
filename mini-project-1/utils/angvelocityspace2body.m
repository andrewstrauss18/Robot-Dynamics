function omega_b = angvelocityspace2body(omega_s,R)
    omega_b = pinv(R)*omega_s;
end
