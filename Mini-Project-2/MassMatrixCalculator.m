function MassM = MassMatrixCalculator(currentQ, S, M, G)

%MassM = zeros(size(currentQ))';
MassM = [];

M0ToEnd = eye(4);

for i = 1:(size(currentQ)+1)
    M0ToEnd = M0ToEnd*M(:,:,i);
end

J = jacobe(S, M0ToEnd, currentQ)

for i = 1:size(S,2)
    Jbi = J(:,i)
    
    MassM = [MassM, Jbi'*G(:,:,i)*Jbi];
    disp(G(:,:,i))
end

end

function J_b = jacobe(S,M,q)    
    J_s = jacob0(S,q);
    T = fkine(S,M,q,'space');
    J_b = adjoint(J_s, pinv(T));
end
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
function T = twist2ht(S,theta)
    omega = S(1:3,:);
    v = S(4:6,:);
    skewOmega = skew(omega);
    R = axisangle2rot(omega, theta);
    P = (eye(3)*theta + (1 - cos(theta))*skewOmega + (theta - sin(theta))*skewOmega^2)*v;
    T = [R     P;
         0 0 0 1];
end
function S = skew(v)
%SKEW Returns the skew-symmetric matrix associated with the 3D vector
%passed as input

    if length(v) == 3
        S = [  0   -v(3)  v(2)
            v(3)  0    -v(1)
            -v(2) v(1)   0];
    else
        error('argument must be a 3-vector');
    end

end
function R = axisangle2rot(omega,theta)
    skewOmega = skew(omega);
    R = eye(3) + sin(theta)*skewOmega + (1 - cos(theta))*skewOmega^2;
end
function [R, p] = TransToRp(T)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes the transformation matrix T in SE(3) 
% Returns R: the corresponding rotation matrix
%         p: the corresponding position vector .
% Example Input:
% 
% clear; clc;
% T = [[1, 0, 0, 0]; [0, 0, -1, 0]; [0, 1, 0, 3]; [0, 0, 0, 1]];
% [R, p] = TransToRp(T)
% 
% Output:
% R =
%     1     0     0
%     0     0    -1
%     0     1     0
% p =
%     0
%     0
%     3

R = T(1: 3, 1: 3);
p = T(1: 3, 4);
end