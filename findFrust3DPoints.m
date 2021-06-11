function frust3DPoints = findFrust3DPoints(W,H,T)
% frust3DPoints = findFrust3DPoints(W,H,T): find the 3D corners of the
% camera view plane
    
    im2DPoints = [1    W    1     W
                  1    1    H     H
                  1    1    1     1  ];
              
    frust3DPoints = T \ im2DPoints; % inv(T)*im2DPoints
end