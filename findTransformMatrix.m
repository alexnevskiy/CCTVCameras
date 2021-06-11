% Copyright 2014 Adina Stoica
% T = findTransformMatrix(W,H,flen,pan,tilt,roll): get the Image to Camera transformation matrix
% Inputs:
% * W,H the width and height of the original camera image
% * flen the focal length of the camera (as well as the distance between the camera and the image center)
%   (Note: All angles below move clockwise: NESW, down-forward-up, botton-left to top-right)
% * pan is the angle of rotation around Z from North [left-right] corresponds to azimuth: 0<=pan<360
% * tilt is the angle of rotation around X from Down [down-up], corresponds to zenith: 0<=tilt<=180
% * roll is the angle of rotation around Y clockwise [side-to-side], (depends on horizon line) 0<=roll<360
%
% Outputs:
% * T is the PlaneToCamection matrix that gets the points from 2D to 3D
%
% pts3D = T \ ptsIm; 
% ptsIm = T * pts3D;
function R = findTransformMatrix(W,H,flen,pan,tilt,roll)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PROCESS THE INPUTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% correct the variables and convert to radians
pan = pan*pi/180;   % the angle of rotation around Z
tilt=180-tilt;
tilt = tilt*pi/180; % the angle of rotation around X
roll = roll*pi/180;  % the angle of rotation around Y

% calibration matrix
K = [flen    0      (W+1)/2
     0       flen   (H+1)/2
     0       0      1  ];
   
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


% the 2D to 3D transformation matrix, corrected by the focal length
R=Rx*Ry*Rz;
%T = K*R / flen;
