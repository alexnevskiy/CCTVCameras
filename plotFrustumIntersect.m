function plotFrustumIntersect(W,H,pan,tilt,roll,fovH,fovV,nearClipPlane,middleClipPlane,farClipPlane,camPos,length)
    % aspect = W / H;         % Соотношение сторон камеры
    
    fovHTan = tan(fovH / 2 / 180 * pi);
    fovVTan = tan(fovV / 2 / 180 * pi);
    % halfHeight = nearClipPlane * fovVTan;
    % halfWidth =  halfHeight / aspect;

    R = findRotationMatrix(pan,tilt,roll);      % Матрица поворота
    X = [0 0 1];                                % Вектор нормали (также просто ненулевой вектор)
    T = X * R;                                  % Вектор направления камеры

    ncpCenter = camPos + T * nearClipPlane;     % Центр основания ближнего frustum
    mcpCenter = camPos + T * middleClipPlane;   % Центр основания среднего frustum
    fcpCenter = camPos + T * farClipPlane;      % Центр основания дальнего frustum

    % Координаты точек основания frustum
    [upRightNear,upLeftNear,downRightNear,downLeftNear] = findFrustumBase(ncpCenter,fovHTan,fovVTan,R,nearClipPlane);
    [upRightMid,upLeftMid,downRightMid,downLeftMid] = findFrustumBase(mcpCenter,fovHTan,fovVTan,R,middleClipPlane);
    [upRightFar,upLeftFar,downRightFar,downLeftFar] = findFrustumBase(fcpCenter,fovHTan,fovVTan,R,farClipPlane);
    
    A = [-length -length 0];    % Координаты точек пола
    B = [-length length 0];
    C = [length length 0];
    D = [length -length 0];
    V0 = [0 1 0];               % Любая точка на плоскости пола

    % Нахождение точек пересечения frustum с плоскостью пола
    [posUpRightNear,checkUpRightNear,posUpLeftNear,checkUpLeftNear,posDownRightNear,checkDownRightNear,...
        posDownLeftNear,checkDownLeftNear] = planeFrustumIntersect(X,V0,camPos,upRightNear,upLeftNear,downRightNear,downLeftNear);
    [posUpRightMid,checkUpRightMid,posUpLeftMid,checkUpLeftMid,posDownRightMid,checkDownRightMid,...
        posDownLeftMid,checkDownLeftMid] = planeFrustumIntersect(X,V0,camPos,upRightMid,upLeftMid,downRightMid,downLeftMid);
    [posUpRightFar,checkUpRightFar,posUpLeftFar,checkUpLeftFar,posDownRightFar,checkDownRightFar,...
        posDownLeftFar,checkDownLeftFar] = planeFrustumIntersect(X,V0,camPos,upRightFar,upLeftFar,downRightFar,downLeftFar);

    f = figure;
    nColor = 'red';
    mColor = 'yellow';
    fColor = 'green';

    % Если рёбра frustum пересекают плоскость пола, то строим пересечение на
    % графике
    interNear = plotPlaneFrustumIntersect(posUpRightNear,checkUpRightNear,posUpLeftNear,...
        checkUpLeftNear,posDownRightNear,checkDownRightNear,...
        posDownLeftNear,checkDownLeftNear,X,V0,downRightNear,upRightNear,downLeftNear,upLeftNear,nColor);
    interMid = plotPlaneFrustumIntersect(posUpRightMid,checkUpRightMid,posUpLeftMid,...
        checkUpLeftMid,posDownRightMid,checkDownRightMid,...
        posDownLeftMid,checkDownLeftMid,X,V0,downRightMid,upRightMid,downLeftMid,upLeftMid,mColor);
    interFar = plotPlaneFrustumIntersect(posUpRightFar,checkUpRightFar,posUpLeftFar,...
        checkUpLeftFar,posDownRightFar,checkDownRightFar,...
        posDownLeftFar,checkDownLeftFar,X,V0,downRightFar,upRightFar,downLeftFar,upLeftFar,fColor);

    % Построение frustum`ов
    PNear = [upRightNear; upLeftNear; downLeftNear; downRightNear; camPos];
    indUpF = [1 2 5]; frustumUpNear = fill3(PNear(indUpF, 1), PNear(indUpF, 2), PNear(indUpF, 3), nColor);
    indLeftF = [2 3 5]; frustumLeftNear = fill3(PNear(indLeftF, 1), PNear(indLeftF, 2), PNear(indLeftF, 3), nColor);
    indDownF = [3 4 5]; frustumDownNear = fill3(PNear(indDownF, 1), PNear(indDownF, 2), PNear(indDownF, 3), nColor);
    indRightF = [4 1 5]; frustumRightNear = fill3(PNear(indRightF, 1), PNear(indRightF, 2), PNear(indRightF, 3), nColor);
    hold on
    
    indUp = [1 2 6 5];
    indLeft = [2 3 7 6]; 
    indDown = [3 4 8 7]; 
    indRight = [4 1 5 8]; 
    
    PMid = [upRightNear; upLeftNear; downLeftNear; downRightNear; ...
        upRightMid; upLeftMid; downLeftMid; downRightMid];
    frustumUpMid = fill3(PMid(indUp, 1), PMid(indUp, 2), PMid(indUp, 3), mColor);
    frustumLeftMid = fill3(PMid(indLeft, 1), PMid(indLeft, 2), PMid(indLeft, 3), mColor);
    frustumDownMid = fill3(PMid(indDown, 1), PMid(indDown, 2), PMid(indDown, 3), mColor);
    frustumRightMid = fill3(PMid(indRight, 1), PMid(indRight, 2), PMid(indRight, 3), mColor);
    hold on
    
    PFar = [upRightMid; upLeftMid; downLeftMid; downRightMid; ...
        upRightFar; upLeftFar; downLeftFar; downRightFar];
    frustumUpFar = fill3(PFar(indUp, 1), PFar(indUp, 2), PFar(indUp, 3), fColor);
    frustumLeftFar = fill3(PFar(indLeft, 1), PFar(indLeft, 2), PFar(indLeft, 3), fColor);
    frustumDownFar = fill3(PFar(indDown, 1), PFar(indDown, 2), PFar(indDown, 3), fColor);
    frustumRightFar = fill3(PFar(indRight, 1), PFar(indRight, 2), PFar(indRight, 3), fColor);
    hold on

    % Построение плоскости пола
    allPlainPts = [A;B;C;D];
    fill3(allPlainPts(:,1),allPlainPts(:,2),allPlainPts(:,3),'white');
    alpha(0.2);
    hold on

    % Построение камеры
    pose = rigid3d(R,camPos);
    cam = plotCamera('AbsolutePose',pose,'Opacity',0, 'AxesVisible', false);

    grid on
    axis equal
    axis manual
    
    distV = 75;
    distH = 450;

    % Поля для изменения параметров на графике
    bgcolor = f.Color;
    bPan = uicontrol('Parent', f, 'Style', 'slider', 'Position', [81,54,419,23],...
                  'Value', pan, 'min', 0, 'max', 360, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[50,54,23,23],...
                    'String','0','BackgroundColor',bgcolor);
    uicontrol('Parent',f,'Style','text','Position',[500,54,23,23],...
                    'String','360','BackgroundColor',bgcolor);
    uicontrol('Parent',f,'Style','text','Position',[240,25,100,23],...
                    'String','Pan','BackgroundColor',bgcolor);
              
    bTilt = uicontrol('Parent', f, 'Style', 'slider', 'Position', [81,54+distV,419,23],...
                  'Value', tilt, 'min', 0, 'max', 90, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[50,54+distV,23,23],...
                    'String','0','BackgroundColor',bgcolor);
    uicontrol('Parent',f,'Style','text','Position',[500,54+distV,23,23],...
                    'String','90','BackgroundColor',bgcolor);
    uicontrol('Parent',f,'Style','text','Position',[240,25+distV,100,23],...
                    'String','Tilt','BackgroundColor',bgcolor); 
                
    bRoll = uicontrol('Parent', f, 'Style', 'slider', 'Position', [81,54+distV*2,419,23],...
                  'Value', roll, 'min', 0, 'max', 360, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[50,54+distV*2,23,23],...
                    'String','0','BackgroundColor',bgcolor);
    uicontrol('Parent',f,'Style','text','Position',[500,54+distV*2,23,23],...
                    'String','360','BackgroundColor',bgcolor);
    uicontrol('Parent',f,'Style','text','Position',[240,25+distV*2,100,23],...
                    'String','Roll (unused)','BackgroundColor',bgcolor);
                
    bCamPosX = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81+distH,54,50,23],...
                  'Value', camPos(1), 'String', camPos(1), 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81+distH,25,50,23],...
                    'String','Camera X','BackgroundColor',bgcolor);

    bCamPosY = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81+distH,54+distV,50,23],...
                  'Value', camPos(2), 'String', camPos(2), 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81+distH,25+distV,50,23],...
                    'String','Camera Y','BackgroundColor',bgcolor);
    
    bCamPosZ = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81+distH,54+distV*2,50,23],...
                  'Value', camPos(3), 'String', camPos(3), 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81+distH,25+distV*2,50,23],...
                    'String','Camera Z','BackgroundColor',bgcolor);
    
    bNearClipPlane = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*2+distH,54,50,23],...
                  'Value', nearClipPlane, 'String', nearClipPlane, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81*2+distH,25,50,23],...
                    'String','Near Clip Plane','BackgroundColor',bgcolor);
                
    bMidClipPlane = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*2+distH,54+distV,50,23],...
                  'Value', middleClipPlane, 'String', middleClipPlane, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81*2+distH,25+distV,50,23],...
                    'String','Mid Clip Plane','BackgroundColor',bgcolor);
    
    bFarClipPlane = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*2+distH,54+distV*2,50,23],...
                  'Value', farClipPlane, 'String', farClipPlane, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81*2+distH,25+distV*2,50,23],...
                    'String','Far Clip Plane','BackgroundColor',bgcolor);
                
    bFovH = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*3+distH,54,50,23],...
                  'Value', fovH, 'String', fovH, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81*3+distH,25,50,23],...
                    'String','FOV Horizontal','BackgroundColor',bgcolor);
    
    bFovV = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*3+distH,54+distV,50,23],...
                  'Value', fovV, 'String', fovV, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81*3+distH,25+distV,50,23],...
                    'String','FOV Vertical','BackgroundColor',bgcolor);

    function update3DPointS(~,~)
        pan = get(bPan,'Value');
        tilt = get(bTilt,'Value');
        roll = get(bRoll,'Value');
        camPosX = str2double(get(bCamPosX,'String'));
        camPosY = str2double(get(bCamPosY,'String'));
        camPosZ = str2double(get(bCamPosZ,'String'));
        nearClipPlane = str2double(get(bNearClipPlane,'String'));
        middleClipPlane = str2double(get(bMidClipPlane,'String'));
        farClipPlane = str2double(get(bFarClipPlane,'String'));
        fovH = str2double(get(bFovH,'String'));
        fovV = str2double(get(bFovV,'String'));
        
        camPos = [camPosX camPosY camPosZ];

        fovHTan = tan(fovH / 2 / 180 * pi);
        fovVTan = tan(fovV / 2 / 180 * pi);
        
        R = findRotationMatrix(pan,tilt,roll);
        pose = rigid3d(R,camPos);
        cam.AbsolutePose = pose; 

        X = [0 0 1];                                % Вектор нормали (также просто ненулевой вектор)
        T = X * R;                                  % Вектор направления камеры

        ncpCenter = camPos + T * nearClipPlane;     % Центр основания frustum
        mcpCenter = camPos + T * middleClipPlane;
        fcpCenter = camPos + T * farClipPlane;

        % Координаты точек основания frustum
        [upRightNear,upLeftNear,downRightNear,downLeftNear] = findFrustumBase(ncpCenter,fovHTan,fovVTan,R,nearClipPlane);
        [upRightMid,upLeftMid,downRightMid,downLeftMid] = findFrustumBase(mcpCenter,fovHTan,fovVTan,R,middleClipPlane);
        [upRightFar,upLeftFar,downRightFar,downLeftFar] = findFrustumBase(fcpCenter,fovHTan,fovVTan,R,farClipPlane);
        
        % Нахождение точек пересечения frustum с плоскостью пола
        [posUpRightNear,checkUpRightNear,posUpLeftNear,checkUpLeftNear,posDownRightNear,checkDownRightNear,...
            posDownLeftNear,checkDownLeftNear] = planeFrustumIntersect(X,V0,camPos,upRightNear,upLeftNear,downRightNear,downLeftNear);
        [posUpRightMid,checkUpRightMid,posUpLeftMid,checkUpLeftMid,posDownRightMid,checkDownRightMid,...
            posDownLeftMid,checkDownLeftMid] = planeFrustumIntersect(X,V0,camPos,upRightMid,upLeftMid,downRightMid,downLeftMid);
        [posUpRightFar,checkUpRightFar,posUpLeftFar,checkUpLeftFar,posDownRightFar,checkDownRightFar,...
            posDownLeftFar,checkDownLeftFar] = planeFrustumIntersect(X,V0,camPos,upRightFar,upLeftFar,downRightFar,downLeftFar);

        % Если рёбра frustum пересекают плоскость пола, то строим пересечение на
        % графике
        interNear = plotSetPlaneFrustumIntersect(posUpRightNear,checkUpRightNear,posUpLeftNear,...
        checkUpLeftNear,posDownRightNear,checkDownRightNear,...
        posDownLeftNear,checkDownLeftNear,X,V0,downRightNear,upRightNear,downLeftNear,upLeftNear,interNear);
        interMid = plotSetPlaneFrustumIntersect(posUpRightMid,checkUpRightMid,posUpLeftMid,...
        checkUpLeftMid,posDownRightMid,checkDownRightMid,...
        posDownLeftMid,checkDownLeftMid,X,V0,downRightMid,upRightMid,downLeftMid,upLeftMid,interMid);
        interFar = plotSetPlaneFrustumIntersect(posUpRightFar,checkUpRightFar,posUpLeftFar,...
        checkUpLeftFar,posDownRightFar,checkDownRightFar,...
        posDownLeftFar,checkDownLeftFar,X,V0,downRightFar,upRightFar,downLeftFar,upLeftFar,interFar);

        % Переопределение построенных frustum`ов
        PNear = [upRightNear; upLeftNear; downLeftNear; downRightNear; camPos];
        set(frustumUpNear, 'XData', PNear(indUpF, 1), 'YData', PNear(indUpF, 2), 'ZData', PNear(indUpF, 3));
        set(frustumLeftNear, 'XData', PNear(indLeftF, 1), 'YData', PNear(indLeftF, 2), 'ZData', PNear(indLeftF, 3));
        set(frustumDownNear, 'XData', PNear(indDownF, 1), 'YData', PNear(indDownF, 2), 'ZData', PNear(indDownF, 3));
        set(frustumRightNear, 'XData', PNear(indRightF, 1), 'YData', PNear(indRightF, 2), 'ZData', PNear(indRightF, 3));
        
        PMid = [upRightNear; upLeftNear; downLeftNear; downRightNear; ...
            upRightMid; upLeftMid; downLeftMid; downRightMid];
        set(frustumUpMid, 'XData', PMid(indUp, 1), 'YData', PMid(indUp, 2), 'ZData', PMid(indUp, 3));
        set(frustumLeftMid, 'XData', PMid(indLeft, 1), 'YData', PMid(indLeft, 2), 'ZData', PMid(indLeft, 3));
        set(frustumDownMid, 'XData', PMid(indDown, 1), 'YData', PMid(indDown, 2), 'ZData', PMid(indDown, 3));
        set(frustumRightMid, 'XData', PMid(indRight, 1), 'YData', PMid(indRight, 2), 'ZData', PMid(indRight, 3));

        PFar = [upRightMid; upLeftMid; downLeftMid; downRightMid; ...
            upRightFar; upLeftFar; downLeftFar; downRightFar];
        set(frustumUpFar, 'XData', PFar(indUp, 1), 'YData', PFar(indUp, 2), 'ZData', PFar(indUp, 3));
        set(frustumLeftFar, 'XData', PFar(indLeft, 1), 'YData', PFar(indLeft, 2), 'ZData', PFar(indLeft, 3));
        set(frustumDownFar, 'XData', PFar(indDown, 1), 'YData', PFar(indDown, 2), 'ZData', PFar(indDown, 3));
        set(frustumRightFar, 'XData', PFar(indRight, 1), 'YData', PFar(indRight, 2), 'ZData', PFar(indRight, 3));
        end
end