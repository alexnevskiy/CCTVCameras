function R = findRotationMatrix(pan,tilt,roll)

% correct the variables and convert to radians
pan = pan*pi/180;   % the angle of rotation around Z
tilt=180-tilt;
tilt = tilt*pi/180; % the angle of rotation around X
roll = roll*pi/180;  % the angle of rotation around Y
   
% Rotation matrices
Rz = [  cos(pan),  -sin(pan), 0;
        sin(pan),   cos(pan), 0;
        0,          0,        1];

Ry = [  cos(roll),   0,   sin(roll);
        0,           1,   0       ;
        -sin(roll),  0,   cos(roll)];

Rx = [1,    0,          0         ;
      0,    cos(tilt), -sin(tilt);
      0,    sin(tilt),  cos(tilt)];

R=Rx*Ry*Rz;