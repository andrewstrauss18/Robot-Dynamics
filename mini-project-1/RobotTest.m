clear, clc, close all
addpath('utils');

plotOn = true;

%% Make Robot

robot = mdl_irb140();
q = zeros(1,6);

S = [0 0 1      0      0     0;        
     0 1 0 -0.352      0  0.07; 
     0 1 0 -0.352      0  0.43; 
     0 0 1      0  -0.43     0;
     0 1 0 -0.732      0  0.43; 
     0 0 1      0  -0.43     0]';
M = [1 0  0  0.43;
     0 0 -1     0;
     0 1  0 0.797;
     0 0  0     1];
 

%% Test Code
shape = ["circle" "square" "infinity" "spiral"];
for o = 1 : 10
    for i = 1 : size(shape, 2)
        path = make_path(shape(i));

        currentQ = zeros(1,6);
        targetPose = zeros(6,size(path,2)); % each column of this matrix is a target pose represented by a twist

        x = [];

        for ii = 1 : size(path,2)
            % First calculate the homogeneous transformation matrix representing
            % each target pose
            R = [0 0 -1; 0 1 0; 1 0 0]';
            T = [R path(:,ii); 
                 0 0 0 1];

            % Then perform the matrix logarithm operation to convert transformation
            % matrices into 4x4 elements of se(3)
            t = MatrixLog6(T);

            % Finally, "unpack" this matrix (i.e., perform the inverse of the
            % bracket operator)
            targetPose(:,ii) = [t(3,2) t(1,3) t(2,1) t(1:3,4)']';
        end
        
        
        
        if plotOn
            robot.teach(currentQ);
            title('Inverse Kinematics Test');
            hold on
            scatter3(path(1,:), path(2,:), path(3,:), 'filled');
        end

        for kk = 1 : size(path, 2)
            % Inverse Kinematics
            tic
            currentQ = ikin(S,M,currentQ,targetPose(:,kk));
            T = fkine(S,M,currentQ,'space');
            H = twist2ht(targetPose(:,kk), 1);
            disp('ikine')
            disp(T)
            disp('target')
            disp(H)
            
            x(end+1) = toc;

            if plotOn
                try
                    robot.teach(currentQ);
                    drawnow;
                catch e
                    disp(e)
                    continue;
                end
            end
        end
        disp('done with path')
    end
end
disp('all paths complete')

mean = mean(x)