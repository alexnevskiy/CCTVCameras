% drawCameraView(img,frust3DPoints): draw a camera frustum and an image into it
% 
% Inputs:
% * img is the image we want to view 
% * frust3DPoints are the 3D coordinates of the image plane in the frustum
% 
% Run as:
% img=imread('image.jpg');
% [H,W,~] = size(img);
% % Need to know the camera focal length, pan, tilt and roll, optionally can use 
% % [tilt,roll]=computeTiltAndRoll(imname,flen);
% T = findTransformMatrix(W,H,flen,pan,tilt,roll);
% frust3DPoints = findFrust3DPoints(W,H,T);
% h = drawCameraView(img,frust3DPoints);
% % To put the view in a cardinal coordinate system, run:
% cardinalCoordSys();

%function h = drawCameraView(img,frust3DPoints)
function drawCameraView(frust3DPoints)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % DRAW VIEW FRUSTUM AND IMAGE
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % the camera position is at the origin to draw the frustum pyramid
    frust3DPoints(:,5)=0; 
    
    % draw the frustum
%     vert1 = [1 2 4 3 5 1 5 2 5 4];
    vert1 = [1 2 4 3 1 5 1 5 2 5 4 5 3];
    plot3(frust3DPoints(1,vert1),frust3DPoints(2,vert1),frust3DPoints(3,vert1),'--k');

    % select the points which will be textured with the image
    %vert2 = [1 2 3 4];
    %xD=frust3DPoints(1,vert2);yD=frust3DPoints(2,vert2);zD=frust3DPoints(3,vert2);
    %xD=reshape(xD,2,2)';yD=reshape(yD,2,2)';zD=reshape(zD,2,2)';

    hold on;
    % texture the camera view with the image
    %h=surf(xD,yD,zD,'CData',img,'FaceColor','texturemap');
end
