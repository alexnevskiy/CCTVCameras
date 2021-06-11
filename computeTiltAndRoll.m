% [tilt,roll]=computeTiltAndRoll(imname,flen): given an image and a focal
% length, this program lets the user draw the horizon line and then computes
% the tilt (up-down) and roll (left-right) of the image. This assumes that
% the picture was taken at horizon height (otherwise only roll is valid)
% Check out my findTransformMatrix.m file for details on how these can be
% used
function [tilt,roll]=computeTiltAndRoll(imname,flen)
    img=imread(imname);
    %%
    [H,W,~]=size(img);
    % flen=420.3714;

    % make the figure fullscreen
    figure('units','normalized','outerposition',[0 0 1 1]); 
    % set the render so MATLAB doesn't crash unexpectedly
    set(gcf,'renderer', 'zbuffer'); 
    % show the image
    imagesc(img); title('Please draw a line at the horizon');

    % click on two points to define a line
    [x_hzn,y_hzn]=ginput(2);
    % if your points are out of the image, put them on the image
    % [x_hzn,y_hzn]=setlims(x,y,2,W,H);
    % draw the line between the points
    line([x_hzn(1) x_hzn(2)],[y_hzn(1) y_hzn(2)],'Color','b','Marker','*','LineWidth',2);
%%
    
    % find the roll angle
    roll = atan((y_hzn(2)-y_hzn(1))/(x_hzn(2)-x_hzn(1)));
    
    % find the coordinates of the points so the horizon line is horizontal
    R_plane = [ cos(roll)  -sin(roll);
                sin(roll)  cos(roll)];
    
    xy = R_plane*[x_hzn,y_hzn]';

    % the calibration matrix 
    K = [flen     0      (W+1)/2
         0        flen   (H+1)/2
         0        0      1  ];

    % the center of the horizon line
    centerOfHorizonH = [W/2, xy(2,1), 1]';
    
    % the vector to the center of horizon
    centerOfHorRay = K \ centerOfHorizonH;
    %%
    % get the tilt angle that gets the horizon line on plane Z = 0
    tilt_hrz = atan(centerOfHorRay(2)/centerOfHorRay(3));
    % the tilt is with respect to the image pointing north, not up, so it's
    % actually pi/2 - tilt_hrz
    %
    tilt = pi/2+tilt_hrz; 
    %%
    roll=roll*180/pi;
    tilt=tilt*180/pi;
    %%
    waitfor(msgbox({['Tilt = ' num2str(tilt) ' degrees'],['Roll = ' num2str(roll) ' degrees']}));
    close;
end