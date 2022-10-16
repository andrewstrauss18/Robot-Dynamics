%credit for this function goes to peter corke @ https://github.com/petercorke/robotics-toolbox-matlab/blob/master/models/mdl_irb140.m
function r = mdl_irb140()
    % robot length values (metres)
    d1 = 0.352;
    a1 = 0.070;
    a2 = 0.360;
    d4 = 0.380;
    d6 = 0.065;
    
    % DH parameter table
    %     theta d a alpha
    dh = [0 d1 a1  -pi/2
          0 0  a2  0
          0 0  0   pi/2
          0 d4 0   -pi/2
          0 0  0   pi/2
          0 d6 0   pi/2];
    
   % and build a serial link manipulator
    
    robot = SerialLink(dh, 'name', 'IRB 140', ...
        'manufacturer', 'ABB', 'ikine', 'nooffset'); 
    
    r = robot;
end
