function plotFrustumIntersect(W,H,pan,tilt,roll,fovH,fovV,...
    nearClipPlane,middleClipPlane,farClipPlane,camPos,length,heightLimit)
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

    % Нахождение точек пересечения frustum'ов с плоскостью пола
    planeInterNear = planeFrustumIntersect(X,V0,camPos,upRightNear,upLeftNear,downRightNear,downLeftNear);
    planeInterMid = planeTruncFrustumIntersect(X,V0,upRightNear,upLeftNear,downRightNear,downLeftNear,...
        upRightMid,upLeftMid,downRightMid,downLeftMid);
    planeInterFar = planeTruncFrustumIntersect(X,V0,upRightMid,upLeftMid,downRightMid,downLeftMid,...
        upRightFar,upLeftFar,downRightFar,downLeftFar);
    
    % Определение плоскости ограничения по высоте
    V1 = [0 1 heightLimit];                 % Любая точка на плоскости пола
    X1 = [0 0 heightLimit + 1];             % Вектор нормали плоскости
    
    % Нахождение точек пересечения полного frustum с плоскостью ограничения
    % по высоте
    planeInterLimit = planeFrustumIntersect(X1,V1,camPos,upRightFar,upLeftFar,downRightFar,downLeftFar);
    
    % Нахождение точек пересечения полного frustum с плоскостью пола
    planeInterFloor = planeFrustumIntersect(X,V0,camPos,upRightFar,upLeftFar,downRightFar,downLeftFar);
    
    % Нахождения конъюнкции многоугольников на высоте ограничения и на полу
    conjunction = planesConjunction(planeInterLimit,planeInterFloor);

    f = figure;
    nColor = 'red';
    mColor = 'yellow';
    fColor = 'green';
    conColor = 'black';

    % Если рёбра frustum пересекают плоскость пола, то строим пересечение на
    % графике
    [mNear, ~] = size(planeInterNear);
    if (mNear < 3)
        interNear = fill3([0 0 0], [0 0 0], [0 0 0], nColor);
        hold on
    else
        interNear = fill3(planeInterNear(:,1),planeInterNear(:,2),planeInterNear(:,3),nColor);
        hold on
    end
    
    [mMid, ~] = size(planeInterMid);
    if (mMid < 3)
        interMid = fill3([0 0 0], [0 0 0], [0 0 0], mColor);
        hold on
    else
        interMid = fill3(planeInterMid(:,1),planeInterMid(:,2),planeInterMid(:,3),mColor);
        hold on
    end

    [mFar, ~] = size(planeInterFar);
    if (mFar < 3)
        interFar = fill3([0 0 0], [0 0 0], [0 0 0], fColor);
        hold on
    else
        interFar = fill3(planeInterFar(:,1),planeInterFar(:,2),planeInterFar(:,3),fColor);
        hold on
    end
    
    % Если рёбра полного frustum пересекают плоскость ограничения по
    % высоте, то строим пересечение на графике
    [mHeight, ~] = size(planeInterLimit);
    if (mHeight < 3)
        interHeight = fill3([0 0 0], [0 0 0], [0 0 0], conColor);
        hold on
    else
        interHeight = fill3(planeInterLimit(:,1),planeInterLimit(:,2),planeInterLimit(:,3),conColor);
        hold on
    end
    
    % Если конъюнкция плоскостей - многоугольник, то строим её на графике
    [mCon, ~] = size(conjunction);
    if (mCon < 3)
        interCon = fill3([0 0 0], [0 0 0], [0 0 0], conColor);
        hold on
    else
        interCon = fill3(conjunction(:,1),conjunction(:,2),conjunction(:,3),conColor);
        hold on
    end

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

    bHeightLimit = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*3+distH,54+distV*2,50,23],...
                  'Value', heightLimit, 'String', heightLimit, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81*3+distH,25+distV*2,50,23],...
                    'String','Height Limit','BackgroundColor',bgcolor);
                
    bFrustumNear = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH,54,130,23],...
                   'Value', 1, 'String', 'Near Frustum', 'Callback', {@update3DPointS});
               
    bFrustumMid = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH,54+distV/2,130,23],...
                   'Value', 1, 'String', 'Middle Frustum', 'Callback', {@update3DPointS});

    bFrustumFar = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH,54+distV,130,23],...
                   'Value', 1, 'String', 'Far Frustum', 'Callback', {@update3DPointS});
    
    bInterNear = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH+171,54,130,23],...
                   'Value', 1, 'String', 'Near Intersection', 'Callback', {@update3DPointS});
               
    bInterMid = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH+171,54+distV/2,130,23],...
                   'Value', 1, 'String', 'Middle Intersection', 'Callback', {@update3DPointS});

    bInterFar = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH+171,54+distV,130,23],...
                   'Value', 1, 'String', 'Far Intersection', 'Callback', {@update3DPointS});
    
    bInterHeight = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH+171*2,54,130,23],...
                   'Value', 1, 'String', 'Height Intersection', 'Callback', {@update3DPointS});
               
    bInterCon = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH+171*2,54+distV/2,130,23],...
                   'Value', 1, 'String', 'Conjunction Intersection', 'Callback', {@update3DPointS});
    
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
        heightLimit = str2double(get(bHeightLimit,'String'));
        nearFrustumCheck = get(bFrustumNear,'Value');
        midFrustumCheck = get(bFrustumMid,'Value');
        farFrustumCheck = get(bFrustumFar,'Value');
        nearInterCheck = get(bInterNear,'Value');
        midInterCheck = get(bInterMid,'Value');
        farInterCheck = get(bInterFar,'Value');
        heightInterCheck = get(bInterHeight,'Value');
        conInterCheck = get(bInterCon,'Value');
        
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
        planeInterNear = planeFrustumIntersect(X,V0,camPos,upRightNear,upLeftNear,downRightNear,downLeftNear);
        planeInterMid = planeTruncFrustumIntersect(X,V0,upRightNear,upLeftNear,downRightNear,downLeftNear,...
            upRightMid,upLeftMid,downRightMid,downLeftMid);
        planeInterFar = planeTruncFrustumIntersect(X,V0,upRightMid,upLeftMid,downRightMid,downLeftMid,...
            upRightFar,upLeftFar,downRightFar,downLeftFar);
        
        % Определение плоскости ограничения по высоте
        V1 = [0 1 heightLimit];                 % Любая точка на плоскости пола
        X1 = [0 0 heightLimit + 1];             % Вектор нормали плоскости
        
        % Нахождение точек пересечения полного frustum с плоскостью ограничения
        % по высоте
        planeInterLimit = planeFrustumIntersect(X1,V1,camPos,upRightFar,upLeftFar,downRightFar,downLeftFar);

        % Нахождение точек пересечения полного frustum с плоскостью пола
        planeInterFloor = planeFrustumIntersect(X,V0,camPos,upRightFar,upLeftFar,downRightFar,downLeftFar);

        % Нахождения конъюнкции многоугольников на высоте ограничения и на полу
        conjunction = planesConjunction(planeInterLimit,planeInterFloor);

        % Если рёбра frustum пересекают плоскость пола, то строим пересечение на
        % графике
        if nearInterCheck == 1
            set(interNear, 'visible', 'on');
            [mNear, ~] = size(planeInterNear);
            if (mNear < 3)
                set(interNear, 'XData', [0 0 0], 'YData', [0 0 0], 'ZData', [0 0 0]);
            else
                set(interNear, 'XData', planeInterNear(:,1), 'YData', planeInterNear(:,2), 'ZData', planeInterNear(:,3));
            end
        else
            set(interNear, 'visible', 'off');
        end

        if midInterCheck == 1
            set(interMid, 'visible', 'on');
            [mMid, ~] = size(planeInterMid);
            if (mMid < 3)
                set(interMid, 'XData', [0 0 0], 'YData', [0 0 0], 'ZData', [0 0 0]);
            else
                set(interMid, 'XData', planeInterMid(:,1), 'YData', planeInterMid(:,2), 'ZData', planeInterMid(:,3));
            end
        else
            set(interMid, 'visible', 'off');
        end

        if farInterCheck == 1
            set(interFar, 'visible', 'on');
            [mFar, ~] = size(planeInterFar);
            if (mFar < 3)
                set(interFar, 'XData', [0 0 0], 'YData', [0 0 0], 'ZData', [0 0 0]);
            else
                set(interFar, 'XData', planeInterFar(:,1), 'YData', planeInterFar(:,2), 'ZData', planeInterFar(:,3));
            end
        else
            set(interFar, 'visible', 'off');
        end
        
        % Если рёбра полного frustum пересекают плоскость ограничения по
        % высоте, то строим пересечение на графике
        if heightInterCheck == 1
            set(interHeight, 'visible', 'on');
            [mHeight, ~] = size(planeInterLimit);
            if (mHeight < 3)
                set(interHeight, 'XData', [0 0 0], 'YData', [0 0 0], 'ZData', [0 0 0]);
            else
                set(interHeight, 'XData', planeInterLimit(:,1), 'YData', planeInterLimit(:,2), 'ZData', planeInterLimit(:,3));
            end
        else
            set(interHeight, 'visible', 'off');
        end

        % Если конъюнкция плоскостей - многоугольник, то строим её на графике
        if conInterCheck == 1
            set(interCon, 'visible', 'on');
            [mCon, ~] = size(conjunction);
            if (mCon < 3)
                set(interCon, 'XData', [0 0 0], 'YData', [0 0 0], 'ZData', [0 0 0]);
            else
                set(interCon, 'XData', conjunction(:,1), 'YData', conjunction(:,2), 'ZData', conjunction(:,3));
            end
        else
            set(interCon, 'visible', 'off');
        end

        % Переопределение построенных frustum`ов
        if nearFrustumCheck == 1
            set(frustumUpNear, 'visible', 'on');
            set(frustumLeftNear, 'visible', 'on');
            set(frustumDownNear, 'visible', 'on');
            set(frustumRightNear, 'visible', 'on');
            PNear = [upRightNear; upLeftNear; downLeftNear; downRightNear; camPos];
            set(frustumUpNear, 'XData', PNear(indUpF, 1), 'YData', PNear(indUpF, 2), 'ZData', PNear(indUpF, 3));
            set(frustumLeftNear, 'XData', PNear(indLeftF, 1), 'YData', PNear(indLeftF, 2), 'ZData', PNear(indLeftF, 3));
            set(frustumDownNear, 'XData', PNear(indDownF, 1), 'YData', PNear(indDownF, 2), 'ZData', PNear(indDownF, 3));
            set(frustumRightNear, 'XData', PNear(indRightF, 1), 'YData', PNear(indRightF, 2), 'ZData', PNear(indRightF, 3));
        else
            set(frustumUpNear, 'visible', 'off');
            set(frustumLeftNear, 'visible', 'off');
            set(frustumDownNear, 'visible', 'off');
            set(frustumRightNear, 'visible', 'off');
        end
        
        if midFrustumCheck == 1
            set(frustumUpMid, 'visible', 'on');
            set(frustumLeftMid, 'visible', 'on');
            set(frustumDownMid, 'visible', 'on');
            set(frustumRightMid, 'visible', 'on');
            PMid = [upRightNear; upLeftNear; downLeftNear; downRightNear; ...
                upRightMid; upLeftMid; downLeftMid; downRightMid];
            set(frustumUpMid, 'XData', PMid(indUp, 1), 'YData', PMid(indUp, 2), 'ZData', PMid(indUp, 3));
            set(frustumLeftMid, 'XData', PMid(indLeft, 1), 'YData', PMid(indLeft, 2), 'ZData', PMid(indLeft, 3));
            set(frustumDownMid, 'XData', PMid(indDown, 1), 'YData', PMid(indDown, 2), 'ZData', PMid(indDown, 3));
            set(frustumRightMid, 'XData', PMid(indRight, 1), 'YData', PMid(indRight, 2), 'ZData', PMid(indRight, 3));
        else
            set(frustumUpMid, 'visible', 'off');
            set(frustumLeftMid, 'visible', 'off');
            set(frustumDownMid, 'visible', 'off');
            set(frustumRightMid, 'visible', 'off');
        end

        if farFrustumCheck == 1
            set(frustumUpFar, 'visible', 'on');
            set(frustumLeftFar, 'visible', 'on');
            set(frustumDownFar, 'visible', 'on');
            set(frustumRightFar, 'visible', 'on');
            PFar = [upRightMid; upLeftMid; downLeftMid; downRightMid; ...
                upRightFar; upLeftFar; downLeftFar; downRightFar];
            set(frustumUpFar, 'XData', PFar(indUp, 1), 'YData', PFar(indUp, 2), 'ZData', PFar(indUp, 3));
            set(frustumLeftFar, 'XData', PFar(indLeft, 1), 'YData', PFar(indLeft, 2), 'ZData', PFar(indLeft, 3));
            set(frustumDownFar, 'XData', PFar(indDown, 1), 'YData', PFar(indDown, 2), 'ZData', PFar(indDown, 3));
            set(frustumRightFar, 'XData', PFar(indRight, 1), 'YData', PFar(indRight, 2), 'ZData', PFar(indRight, 3));
        else
            set(frustumUpFar, 'visible', 'off');
            set(frustumLeftFar, 'visible', 'off');
            set(frustumDownFar, 'visible', 'off');
            set(frustumRightFar, 'visible', 'off');
        end
    end
end