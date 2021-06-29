function plotFrustumIntersect(W,H,pan,tilt,roll,fovH,fovV,...
    nearClipPlane,middleClipPlane,farClipPlane,camPos,length,heightLimit,numberOfObjects)
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
    
    % Нахождение конъюнкции многоугольников на высоте ограничения и на полу
    conjunction = planesConjunction(planeInterLimit,planeInterFloor);
    
    % Создание объектов и нахождение слепых зон
    interNearPoly = polyshape(planeInterNear(:,1:2));
    interMidPoly = polyshape(planeInterMid(:,1:2));
    interFarPoly = polyshape(planeInterFar(:,1:2));
    
    maxNumberOfObjects = 5;
    parallelepipeds = cell(maxNumberOfObjects,1);
    
    floorPoly = polyshape(planeInterFloor(:,1:2));

    for i = 1:maxNumberOfObjects
        parallelepipeds{i,1} = getParallelepiped(i,i,i*2,[i*2 i*2 0]);
        if ((i <= numberOfObjects) && (~isBehind(parallelepipeds{i,1},T,camPos,'any')))
            if (isBehind(parallelepipeds{i,1},X,V0,'full'))
                continue
            elseif (isBehind(parallelepipeds{i,1},X,V0,'any'))
                TRInter = planeObjectIntersection(parallelepipeds{i,1},X,V0);
                TRPlane = planeProjection(TRInter,X,V0,T,camPos,upRightFar,upLeftFar);
            else
                TRPlane = planeProjection(parallelepipeds{i,1},X,V0,T,camPos,upRightFar,upLeftFar);
            end
            unPoly = unionPolygons(TRPlane);
            floorPoly = subtract(floorPoly,unPoly);
        end
    end

    if ((numberOfObjects > 0) && (numberOfObjects <= maxNumberOfObjects))
        interNearPoly = intersect(interNearPoly,floorPoly);
        interMidPoly = intersect(interMidPoly,floorPoly);
        interFarPoly = intersect(interFarPoly,floorPoly);
    end
    
    f = figure;
    nColor = 'red';
    mColor = 'yellow';
    fColor = 'green';
    conColor = 'black';
    paralColor = 'blue';

    % Если рёбра frustum пересекают плоскость пола, то строим пересечение на
    % графике (вместе со слепыми зонами)
    [mNear, ~] = size(interNearPoly.Vertices);
    if (mNear < 3)
        interNear = plot(polyshape([0 0 0 0], [0 0 0 0]),'FaceColor',nColor);
        hold on
    else
        interNear = plot(interNearPoly,'FaceColor',nColor);
        hold on
    end
    
    [mMid, ~] = size(interMidPoly.Vertices);
    if (mMid < 3)
        interMid = plot(polyshape([0 0 0 0], [0 0 0 0]),'FaceColor',mColor);
        hold on
    else
        interMid = plot(interMidPoly,'FaceColor',mColor);
        hold on
    end

    [mFar, ~] = size(interFarPoly.Vertices);
    if (mFar < 3)
        interFar = plot(polyshape([0 0 0 0], [0 0 0 0]),'FaceColor',fColor);
        hold on
    else
        interFar = plot(interFarPoly,'FaceColor',fColor);
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
    
    % Построение объектов (препятствий) на графике
    paralSurf = cell(maxNumberOfObjects,1);
    
    for i = 1:maxNumberOfObjects
        paralSurf{i,1} = trisurf(parallelepipeds{i,1},'FaceColor',paralColor);
        hold on;
        if i > numberOfObjects
            paralSurf{i,1}.Visible = 'off';
        end
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
    view(3);
    
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
                
    bFrustumNear = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH,54-distV/2,130,23],...
                   'Value', 1, 'String', 'Near Frustum', 'Callback', {@update3DPointS});
               
    bFrustumMid = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH,54,130,23],...
                   'Value', 1, 'String', 'Middle Frustum', 'Callback', {@update3DPointS});

    bFrustumFar = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH,54+distV/2,130,23],...
                   'Value', 1, 'String', 'Far Frustum', 'Callback', {@update3DPointS});
    
    bInterNear = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH,54+distV,130,23],...
                   'Value', 1, 'String', 'Near Intersection', 'Callback', {@update3DPointS});
               
    bInterMid = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH,54+distV*3/2,130,23],...
                   'Value', 1, 'String', 'Middle Intersection', 'Callback', {@update3DPointS});

    bInterFar = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH,54+distV*2,130,23],...
                   'Value', 1, 'String', 'Far Intersection', 'Callback', {@update3DPointS});
    
    bInterHeight = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH+161,54-distV/2,130,23],...
                   'Value', 1, 'String', 'Height Intersection', 'Callback', {@update3DPointS});
               
    bInterCon = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH+161,54,130,23],...
                   'Value', 1, 'String', 'Conjunction Intersection', 'Callback', {@update3DPointS});
    
    bNumberOfObjects = uicontrol('Parent', f, 'Style', 'popupmenu', 'Position', [81*4+distH+251,54+distV*3/2,50,23],...
                   'Value', numberOfObjects + 1, 'String', {'0','1','2','3','4','5'}, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81*4+distH+161,54+distV*3/2,85,23],...
                    'String','Number of objects','BackgroundColor',bgcolor);
                
    bObjectX1 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*4+distH+161*2,54,50,23],...
                  'Value', parallelepipeds{1,1}.Points(7,1), 'String', parallelepipeds{1,1}.Points(7,1), 'Callback', {@update3DPointS});
    bObjectX1T = uicontrol('Parent',f,'Style','text','Position',[81*4+distH+161*2,25,50,23],...
                    'String','1 Object X','BackgroundColor',bgcolor);

    bObjectY1 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*4+distH+161*2,54+distV,50,23],...
                  'Value', parallelepipeds{1,1}.Points(7,2), 'String', parallelepipeds{1,1}.Points(7,2), 'Callback', {@update3DPointS});
    bObjectY1T = uicontrol('Parent',f,'Style','text','Position',[81*4+distH+161*2,25+distV,50,23],...
                    'String','1 Object Y','BackgroundColor',bgcolor);
    
    bObjectZ1 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*4+distH+161*2,54+distV*2,50,23],...
                  'Value', parallelepipeds{1,1}.Points(7,3), 'String', parallelepipeds{1,1}.Points(7,3), 'Callback', {@update3DPointS});
    bObjectZ1T = uicontrol('Parent',f,'Style','text','Position',[81*4+distH+161*2,25+distV*2,50,23],...
                    'String','1 Object Z','BackgroundColor',bgcolor);
    
    bObjectWidth1 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*5+distH+161*2,54,50,23],...
                  'Value', parallelepipeds{1,1}.Points(4,1) - parallelepipeds{1,1}.Points(7,1),...
                  'String', parallelepipeds{1,1}.Points(4,1) - parallelepipeds{1,1}.Points(7,1), 'Callback', {@update3DPointS});
    bObjectWidth1T = uicontrol('Parent',f,'Style','text','Position',[81*5+distH+161*2,25,50,25],...
                    'String','1 Object Width','BackgroundColor',bgcolor);

    bObjectDepth1 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*5+distH+161*2,54+distV,50,23],...
                  'Value', parallelepipeds{1,1}.Points(4,2) - parallelepipeds{1,1}.Points(7,2),...
                  'String', parallelepipeds{1,1}.Points(4,2) - parallelepipeds{1,1}.Points(7,2), 'Callback', {@update3DPointS});
    bObjectDepth1T = uicontrol('Parent',f,'Style','text','Position',[81*5+distH+161*2,25+distV,50,25],...
                    'String','1 Object Depth','BackgroundColor',bgcolor);
    
    bObjectHeight1 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*5+distH+161*2,54+distV*2,50,23],...
                  'Value', parallelepipeds{1,1}.Points(4,3) - parallelepipeds{1,1}.Points(7,3),...
                  'String', parallelepipeds{1,1}.Points(4,3) - parallelepipeds{1,1}.Points(7,3), 'Callback', {@update3DPointS});
    bObjectHeight1T = uicontrol('Parent',f,'Style','text','Position',[81*5+distH+161*2,25+distV*2,50,25],...
                    'String','1 Object Height','BackgroundColor',bgcolor);
    
    bObjectX2 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*6+distH+161*2,54,50,23],...
                  'Value', parallelepipeds{2,1}.Points(7,1), 'String', parallelepipeds{2,1}.Points(7,1), 'Callback', {@update3DPointS});
    bObjectX2T = uicontrol('Parent',f,'Style','text','Position',[81*6+distH+161*2,25,50,23],...
                    'String','2 Object X','BackgroundColor',bgcolor);

    bObjectY2 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*6+distH+161*2,54+distV,50,23],...
                  'Value', parallelepipeds{2,1}.Points(7,2), 'String', parallelepipeds{2,1}.Points(7,2), 'Callback', {@update3DPointS});
    bObjectY2T = uicontrol('Parent',f,'Style','text','Position',[81*6+distH+161*2,25+distV,50,23],...
                    'String','2 Object Y','BackgroundColor',bgcolor);
    
    bObjectZ2 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*6+distH+161*2,54+distV*2,50,23],...
                  'Value', parallelepipeds{2,1}.Points(7,3), 'String', parallelepipeds{2,1}.Points(7,3), 'Callback', {@update3DPointS});
    bObjectZ2T = uicontrol('Parent',f,'Style','text','Position',[81*6+distH+161*2,25+distV*2,50,23],...
                    'String','2 Object Z','BackgroundColor',bgcolor);
    
    bObjectWidth2 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*7+distH+161*2,54,50,23],...
                  'Value', parallelepipeds{2,1}.Points(4,1) - parallelepipeds{2,1}.Points(7,1),...
                  'String', parallelepipeds{2,1}.Points(4,1) - parallelepipeds{2,1}.Points(7,1), 'Callback', {@update3DPointS});
    bObjectWidth2T = uicontrol('Parent',f,'Style','text','Position',[81*7+distH+161*2,25,50,25],...
                    'String','2 Object Width','BackgroundColor',bgcolor);

    bObjectDepth2 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*7+distH+161*2,54+distV,50,23],...
                  'Value', parallelepipeds{2,1}.Points(4,2) - parallelepipeds{2,1}.Points(7,2),...
                  'String', parallelepipeds{2,1}.Points(4,2) - parallelepipeds{2,1}.Points(7,2), 'Callback', {@update3DPointS});
    bObjectDepth2T = uicontrol('Parent',f,'Style','text','Position',[81*7+distH+161*2,25+distV,50,25],...
                    'String','2 Object Depth','BackgroundColor',bgcolor);
    
    bObjectHeight2 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*7+distH+161*2,54+distV*2,50,23],...
                  'Value', parallelepipeds{2,1}.Points(4,3) - parallelepipeds{2,1}.Points(7,3),...
                  'String', parallelepipeds{2,1}.Points(4,3) - parallelepipeds{2,1}.Points(7,3), 'Callback', {@update3DPointS});
    bObjectHeight2T = uicontrol('Parent',f,'Style','text','Position',[81*7+distH+161*2,25+distV*2,50,25],...
                    'String','2 Object Height','BackgroundColor',bgcolor);
    
    bObjectX3 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*8+distH+161*2,54,50,23],...
                  'Value', parallelepipeds{3,1}.Points(7,1), 'String', parallelepipeds{3,1}.Points(7,1), 'Callback', {@update3DPointS});
    bObjectX3T = uicontrol('Parent',f,'Style','text','Position',[81*8+distH+161*2,25,50,23],...
                    'String','3 Object X','BackgroundColor',bgcolor);

    bObjectY3 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*8+distH+161*2,54+distV,50,23],...
                  'Value', parallelepipeds{3,1}.Points(7,2), 'String', parallelepipeds{3,1}.Points(7,2), 'Callback', {@update3DPointS});
    bObjectY3T = uicontrol('Parent',f,'Style','text','Position',[81*8+distH+161*2,25+distV,50,23],...
                    'String','3 Object Y','BackgroundColor',bgcolor);
    
    bObjectZ3 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*8+distH+161*2,54+distV*2,50,23],...
                  'Value', parallelepipeds{3,1}.Points(7,3), 'String', parallelepipeds{3,1}.Points(7,3), 'Callback', {@update3DPointS});
    bObjectZ3T = uicontrol('Parent',f,'Style','text','Position',[81*8+distH+161*2,25+distV*2,50,23],...
                    'String','3 Object Z','BackgroundColor',bgcolor);
    
    bObjectWidth3 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*9+distH+161*2,54,50,23],...
                  'Value', parallelepipeds{3,1}.Points(4,1) - parallelepipeds{3,1}.Points(7,1),...
                  'String', parallelepipeds{3,1}.Points(4,1) - parallelepipeds{3,1}.Points(7,1), 'Callback', {@update3DPointS});
    bObjectWidth3T = uicontrol('Parent',f,'Style','text','Position',[81*9+distH+161*2,25,50,25],...
                    'String','3 Object Width','BackgroundColor',bgcolor);

    bObjectDepth3 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*9+distH+161*2,54+distV,50,23],...
                  'Value', parallelepipeds{3,1}.Points(4,2) - parallelepipeds{3,1}.Points(7,2),...
                  'String', parallelepipeds{3,1}.Points(4,2) - parallelepipeds{3,1}.Points(7,2), 'Callback', {@update3DPointS});
    bObjectDepth3T = uicontrol('Parent',f,'Style','text','Position',[81*9+distH+161*2,25+distV,50,25],...
                    'String','3 Object Depth','BackgroundColor',bgcolor);
    
    bObjectHeight3 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*9+distH+161*2,54+distV*2,50,23],...
                  'Value', parallelepipeds{3,1}.Points(4,3) - parallelepipeds{3,1}.Points(7,3),...
                  'String', parallelepipeds{3,1}.Points(4,3) - parallelepipeds{3,1}.Points(7,3), 'Callback', {@update3DPointS});
    bObjectHeight3T = uicontrol('Parent',f,'Style','text','Position',[81*9+distH+161*2,25+distV*2,50,25],...
                    'String','3 Object Height','BackgroundColor',bgcolor);
    
    bObjectX4 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*10+distH+161*2,54,50,23],...
                  'Value', parallelepipeds{4,1}.Points(7,1), 'String', parallelepipeds{4,1}.Points(7,1), 'Callback', {@update3DPointS});
    bObjectX4T = uicontrol('Parent',f,'Style','text','Position',[81*10+distH+161*2,25,50,23],...
                    'String','4 Object X','BackgroundColor',bgcolor);

    bObjectY4 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*10+distH+161*2,54+distV,50,23],...
                  'Value', parallelepipeds{4,1}.Points(7,2), 'String', parallelepipeds{4,1}.Points(7,2), 'Callback', {@update3DPointS});
    bObjectY4T = uicontrol('Parent',f,'Style','text','Position',[81*10+distH+161*2,25+distV,50,23],...
                    'String','4 Object Y','BackgroundColor',bgcolor);
    
    bObjectZ4 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*10+distH+161*2,54+distV*2,50,23],...
                  'Value', parallelepipeds{4,1}.Points(7,3), 'String', parallelepipeds{4,1}.Points(7,3), 'Callback', {@update3DPointS});
    bObjectZ4T = uicontrol('Parent',f,'Style','text','Position',[81*10+distH+161*2,25+distV*2,50,23],...
                    'String','4 Object Z','BackgroundColor',bgcolor);
    
    bObjectWidth4 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*11+distH+161*2,54,50,23],...
                  'Value', parallelepipeds{4,1}.Points(4,1) - parallelepipeds{4,1}.Points(7,1),...
                  'String', parallelepipeds{4,1}.Points(4,1) - parallelepipeds{4,1}.Points(7,1), 'Callback', {@update3DPointS});
    bObjectWidth4T = uicontrol('Parent',f,'Style','text','Position',[81*11+distH+161*2,25,50,25],...
                    'String','4 Object Width','BackgroundColor',bgcolor);

    bObjectDepth4 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*11+distH+161*2,54+distV,50,23],...
                  'Value', parallelepipeds{4,1}.Points(4,2) - parallelepipeds{4,1}.Points(7,2),...
                  'String', parallelepipeds{4,1}.Points(4,2) - parallelepipeds{4,1}.Points(7,2), 'Callback', {@update3DPointS});
    bObjectDepth4T = uicontrol('Parent',f,'Style','text','Position',[81*11+distH+161*2,25+distV,50,25],...
                    'String','4 Object Depth','BackgroundColor',bgcolor);
    
    bObjectHeight4 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*11+distH+161*2,54+distV*2,50,23],...
                  'Value', parallelepipeds{4,1}.Points(4,3) - parallelepipeds{4,1}.Points(7,3),...
                  'String', parallelepipeds{4,1}.Points(4,3) - parallelepipeds{4,1}.Points(7,3), 'Callback', {@update3DPointS});
    bObjectHeight4T = uicontrol('Parent',f,'Style','text','Position',[81*11+distH+161*2,25+distV*2,50,25],...
                    'String','4 Object Height','BackgroundColor',bgcolor);
    
    bObjectX5 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*12+distH+161*2,54,50,23],...
                  'Value', parallelepipeds{5,1}.Points(7,1), 'String', parallelepipeds{5,1}.Points(7,1), 'Callback', {@update3DPointS});
    bObjectX5T = uicontrol('Parent',f,'Style','text','Position',[81*12+distH+161*2,25,50,23],...
                    'String','5 Object X','BackgroundColor',bgcolor);

    bObjectY5 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*12+distH+161*2,54+distV,50,23],...
                  'Value', parallelepipeds{5,1}.Points(7,2), 'String', parallelepipeds{5,1}.Points(7,2), 'Callback', {@update3DPointS});
    bObjectY5T = uicontrol('Parent',f,'Style','text','Position',[81*12+distH+161*2,25+distV,50,23],...
                    'String','5 Object Y','BackgroundColor',bgcolor);
    
    bObjectZ5 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*12+distH+161*2,54+distV*2,50,23],...
                  'Value', parallelepipeds{5,1}.Points(7,3), 'String', parallelepipeds{5,1}.Points(7,3), 'Callback', {@update3DPointS});
    bObjectZ5T = uicontrol('Parent',f,'Style','text','Position',[81*12+distH+161*2,25+distV*2,50,23],...
                    'String','5 Object Z','BackgroundColor',bgcolor);
    
    bObjectWidth5 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*13+distH+161*2,54,50,23],...
                  'Value', parallelepipeds{5,1}.Points(4,1) - parallelepipeds{5,1}.Points(7,1),...
                  'String', parallelepipeds{5,1}.Points(4,1) - parallelepipeds{5,1}.Points(7,1), 'Callback', {@update3DPointS});
    bObjectWidth5T = uicontrol('Parent',f,'Style','text','Position',[81*13+distH+161*2,25,50,25],...
                    'String','5 Object Width','BackgroundColor',bgcolor);

    bObjectDepth5 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*13+distH+161*2,54+distV,50,23],...
                  'Value', parallelepipeds{5,1}.Points(4,2) - parallelepipeds{5,1}.Points(7,2),...
                  'String', parallelepipeds{5,1}.Points(4,2) - parallelepipeds{5,1}.Points(7,2), 'Callback', {@update3DPointS});
    bObjectDepth5T = uicontrol('Parent',f,'Style','text','Position',[81*13+distH+161*2,25+distV,50,25],...
                    'String','5 Object Depth','BackgroundColor',bgcolor);
    
    bObjectHeight5 = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*13+distH+161*2,54+distV*2,50,23],...
                  'Value', parallelepipeds{5,1}.Points(4,3) - parallelepipeds{5,1}.Points(7,3),...
                  'String', parallelepipeds{5,1}.Points(4,3) - parallelepipeds{5,1}.Points(7,3), 'Callback', {@update3DPointS});
    bObjectHeight5T = uicontrol('Parent',f,'Style','text','Position',[81*13+distH+161*2,25+distV*2,50,25],...
                    'String','5 Object Height','BackgroundColor',bgcolor);
                
    switch numberOfObjects
        case 0
            set(bObjectX1,'Visible','off');
            set(bObjectY1,'Visible','off');
            set(bObjectZ1,'Visible','off');
            set(bObjectWidth1,'Visible','off');
            set(bObjectDepth1,'Visible','off');
            set(bObjectHeight1,'Visible','off');
            set(bObjectX2,'Visible','off');
            set(bObjectY2,'Visible','off');
            set(bObjectZ2,'Visible','off');
            set(bObjectWidth2,'Visible','off');
            set(bObjectDepth2,'Visible','off');
            set(bObjectHeight2,'Visible','off');
            set(bObjectX3,'Visible','off');
            set(bObjectY3,'Visible','off');
            set(bObjectZ3,'Visible','off');
            set(bObjectWidth3,'Visible','off');
            set(bObjectDepth3,'Visible','off');
            set(bObjectHeight3,'Visible','off');
            set(bObjectX4,'Visible','off');
            set(bObjectY4,'Visible','off');
            set(bObjectZ4,'Visible','off');
            set(bObjectWidth4,'Visible','off');
            set(bObjectDepth4,'Visible','off');
            set(bObjectHeight4,'Visible','off');
            set(bObjectX5,'Visible','off');
            set(bObjectY5,'Visible','off');
            set(bObjectZ5,'Visible','off');
            set(bObjectWidth5,'Visible','off');
            set(bObjectDepth5,'Visible','off');
            set(bObjectHeight5,'Visible','off');
            
            set(bObjectX1T,'Visible','off');
            set(bObjectY1T,'Visible','off');
            set(bObjectZ1T,'Visible','off');
            set(bObjectWidth1T,'Visible','off');
            set(bObjectDepth1T,'Visible','off');
            set(bObjectHeight1T,'Visible','off');
            set(bObjectX2T,'Visible','off');
            set(bObjectY2T,'Visible','off');
            set(bObjectZ2T,'Visible','off');
            set(bObjectWidth2T,'Visible','off');
            set(bObjectDepth2T,'Visible','off');
            set(bObjectHeight2T,'Visible','off');
            set(bObjectX3T,'Visible','off');
            set(bObjectY3T,'Visible','off');
            set(bObjectZ3T,'Visible','off');
            set(bObjectWidth3T,'Visible','off');
            set(bObjectDepth3T,'Visible','off');
            set(bObjectHeight3T,'Visible','off');
            set(bObjectX4T,'Visible','off');
            set(bObjectY4T,'Visible','off');
            set(bObjectZ4T,'Visible','off');
            set(bObjectWidth4T,'Visible','off');
            set(bObjectDepth4T,'Visible','off');
            set(bObjectHeight4T,'Visible','off');
            set(bObjectX5T,'Visible','off');
            set(bObjectY5T,'Visible','off');
            set(bObjectZ5T,'Visible','off');
            set(bObjectWidth5T,'Visible','off');
            set(bObjectDepth5T,'Visible','off');
            set(bObjectHeight5T,'Visible','off');
        case 1
            set(bObjectX2,'Visible','off');
            set(bObjectY2,'Visible','off');
            set(bObjectZ2,'Visible','off');
            set(bObjectWidth2,'Visible','off');
            set(bObjectDepth2,'Visible','off');
            set(bObjectHeight2,'Visible','off');
            set(bObjectX3,'Visible','off');
            set(bObjectY3,'Visible','off');
            set(bObjectZ3,'Visible','off');
            set(bObjectWidth3,'Visible','off');
            set(bObjectDepth3,'Visible','off');
            set(bObjectHeight3,'Visible','off');
            set(bObjectX4,'Visible','off');
            set(bObjectY4,'Visible','off');
            set(bObjectZ4,'Visible','off');
            set(bObjectWidth4,'Visible','off');
            set(bObjectDepth4,'Visible','off');
            set(bObjectHeight4,'Visible','off');
            set(bObjectX5,'Visible','off');
            set(bObjectY5,'Visible','off');
            set(bObjectZ5,'Visible','off');
            set(bObjectWidth5,'Visible','off');
            set(bObjectDepth5,'Visible','off');
            set(bObjectHeight5,'Visible','off');
            
            set(bObjectX2T,'Visible','off');
            set(bObjectY2T,'Visible','off');
            set(bObjectZ2T,'Visible','off');
            set(bObjectWidth2T,'Visible','off');
            set(bObjectDepth2T,'Visible','off');
            set(bObjectHeight2T,'Visible','off');
            set(bObjectX3T,'Visible','off');
            set(bObjectY3T,'Visible','off');
            set(bObjectZ3T,'Visible','off');
            set(bObjectWidth3T,'Visible','off');
            set(bObjectDepth3T,'Visible','off');
            set(bObjectHeight3T,'Visible','off');
            set(bObjectX4T,'Visible','off');
            set(bObjectY4T,'Visible','off');
            set(bObjectZ4T,'Visible','off');
            set(bObjectWidth4T,'Visible','off');
            set(bObjectDepth4T,'Visible','off');
            set(bObjectHeight4T,'Visible','off');
            set(bObjectX5T,'Visible','off');
            set(bObjectY5T,'Visible','off');
            set(bObjectZ5T,'Visible','off');
            set(bObjectWidth5T,'Visible','off');
            set(bObjectDepth5T,'Visible','off');
            set(bObjectHeight5T,'Visible','off');
        case 2
            set(bObjectX3,'Visible','off');
            set(bObjectY3,'Visible','off');
            set(bObjectZ3,'Visible','off');
            set(bObjectWidth3,'Visible','off');
            set(bObjectDepth3,'Visible','off');
            set(bObjectHeight3,'Visible','off');
            set(bObjectX4,'Visible','off');
            set(bObjectY4,'Visible','off');
            set(bObjectZ4,'Visible','off');
            set(bObjectWidth4,'Visible','off');
            set(bObjectDepth4,'Visible','off');
            set(bObjectHeight4,'Visible','off');
            set(bObjectX5,'Visible','off');
            set(bObjectY5,'Visible','off');
            set(bObjectZ5,'Visible','off');
            set(bObjectWidth5,'Visible','off');
            set(bObjectDepth5,'Visible','off');
            set(bObjectHeight5,'Visible','off');
            
            set(bObjectX3T,'Visible','off');
            set(bObjectY3T,'Visible','off');
            set(bObjectZ3T,'Visible','off');
            set(bObjectWidth3T,'Visible','off');
            set(bObjectDepth3T,'Visible','off');
            set(bObjectHeight3T,'Visible','off');
            set(bObjectX4T,'Visible','off');
            set(bObjectY4T,'Visible','off');
            set(bObjectZ4T,'Visible','off');
            set(bObjectWidth4T,'Visible','off');
            set(bObjectDepth4T,'Visible','off');
            set(bObjectHeight4T,'Visible','off');
            set(bObjectX5T,'Visible','off');
            set(bObjectY5T,'Visible','off');
            set(bObjectZ5T,'Visible','off');
            set(bObjectWidth5T,'Visible','off');
            set(bObjectDepth5T,'Visible','off');
            set(bObjectHeight5T,'Visible','off');
        case 3
            set(bObjectX4,'Visible','off');
            set(bObjectY4,'Visible','off');
            set(bObjectZ4,'Visible','off');
            set(bObjectWidth4,'Visible','off');
            set(bObjectDepth4,'Visible','off');
            set(bObjectHeight4,'Visible','off');
            set(bObjectX5,'Visible','off');
            set(bObjectY5,'Visible','off');
            set(bObjectZ5,'Visible','off');
            set(bObjectWidth5,'Visible','off');
            set(bObjectDepth5,'Visible','off');
            set(bObjectHeight5,'Visible','off');
            
            set(bObjectX4T,'Visible','off');
            set(bObjectY4T,'Visible','off');
            set(bObjectZ4T,'Visible','off');
            set(bObjectWidth4T,'Visible','off');
            set(bObjectDepth4T,'Visible','off');
            set(bObjectHeight4T,'Visible','off');
            set(bObjectX5T,'Visible','off');
            set(bObjectY5T,'Visible','off');
            set(bObjectZ5T,'Visible','off');
            set(bObjectWidth5T,'Visible','off');
            set(bObjectDepth5T,'Visible','off');
            set(bObjectHeight5T,'Visible','off');
        case 4
            set(bObjectX5,'Visible','off');
            set(bObjectY5,'Visible','off');
            set(bObjectZ5,'Visible','off');
            set(bObjectWidth5,'Visible','off');
            set(bObjectDepth5,'Visible','off');
            set(bObjectHeight5,'Visible','off');
            
            set(bObjectX5T,'Visible','off');
            set(bObjectY5T,'Visible','off');
            set(bObjectZ5T,'Visible','off');
            set(bObjectWidth5T,'Visible','off');
            set(bObjectDepth5T,'Visible','off');
            set(bObjectHeight5T,'Visible','off');
    end
    
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
        numberOfObjects = get(bNumberOfObjects,'Value') - 1;
        
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
        
        % Создание объектов и нахождение слепых зон
        interNearPoly = polyshape(planeInterNear(:,1:2));
        interMidPoly = polyshape(planeInterMid(:,1:2));
        interFarPoly = polyshape(planeInterFar(:,1:2));

        parallelepipeds = cell(maxNumberOfObjects,1);

        floorPoly = polyshape(planeInterFloor(:,1:2));

        for j = 1:maxNumberOfObjects
            switch j
                case 1
                    x = str2double(get(bObjectX1,'String'));
                    y = str2double(get(bObjectY1,'String'));
                    z = str2double(get(bObjectZ1,'String'));
                    width = str2double(get(bObjectWidth1,'String'));
                    depth = str2double(get(bObjectDepth1,'String'));
                    height = str2double(get(bObjectHeight1,'String'));
                case 2
                    x = str2double(get(bObjectX2,'String'));
                    y = str2double(get(bObjectY2,'String'));
                    z = str2double(get(bObjectZ2,'String'));
                    width = str2double(get(bObjectWidth2,'String'));
                    depth = str2double(get(bObjectDepth2,'String'));
                    height = str2double(get(bObjectHeight2,'String'));
                case 3
                    x = str2double(get(bObjectX3,'String'));
                    y = str2double(get(bObjectY3,'String'));
                    z = str2double(get(bObjectZ3,'String'));
                    width = str2double(get(bObjectWidth3,'String'));
                    depth = str2double(get(bObjectDepth3,'String'));
                    height = str2double(get(bObjectHeight3,'String'));
                case 4
                    x = str2double(get(bObjectX4,'String'));
                    y = str2double(get(bObjectY4,'String'));
                    z = str2double(get(bObjectZ4,'String'));
                    width = str2double(get(bObjectWidth4,'String'));
                    depth = str2double(get(bObjectDepth4,'String'));
                    height = str2double(get(bObjectHeight4,'String'));
                case 5
                    x = str2double(get(bObjectX5,'String'));
                    y = str2double(get(bObjectY5,'String'));
                    z = str2double(get(bObjectZ5,'String'));
                    width = str2double(get(bObjectWidth5,'String'));
                    depth = str2double(get(bObjectDepth5,'String'));
                    height = str2double(get(bObjectHeight5,'String'));
            end
            parallelepipeds{j,1} = getParallelepiped(width,depth,height,[x y z]);
            if ((j <= numberOfObjects) && (~isBehind(parallelepipeds{j,1},T,camPos,'any')))
                if (isBehind(parallelepipeds{j,1},X,V0,'full'))
                    continue
                elseif (isBehind(parallelepipeds{j,1},X,V0,'any'))
                    TRInter = planeObjectIntersection(parallelepipeds{j,1},X,V0);
                    TRPlane = planeProjection(TRInter,X,V0,T,camPos,upRightFar,upLeftFar);
                else
                    TRPlane = planeProjection(parallelepipeds{j,1},X,V0,T,camPos,upRightFar,upLeftFar);
                end
                unPoly = unionPolygons(TRPlane);
                floorPoly = subtract(floorPoly,unPoly);
            end
        end

        if ((numberOfObjects > 0) && (numberOfObjects <= maxNumberOfObjects))
            interNearPoly = intersect(interNearPoly,floorPoly);
            interMidPoly = intersect(interMidPoly,floorPoly);
            interFarPoly = intersect(interFarPoly,floorPoly);
        end
        
        % Перемещение polyshape на нужную высоту (на будущее)
        %     M = [   1   0   0   0
        %             0   1   0   0
        %             0   0   1   10
        %             0   0   0   1   ];
        %     t=hgtransform('Matrix',M);     
        %     Hpgon = plot(interFarPoly,'Parent',t,'FaceColor','r');

        % Если рёбра frustum пересекают плоскость пола, то строим пересечение на
        % графике
        if nearInterCheck == 1
            interNear.Visible = 'on';
            [mNear, ~] = size(interNearPoly.Vertices);
            if (mNear < 3)
                interNear.Shape.Vertices = [];
            else
                interNear.Shape.Vertices = interNearPoly.Vertices;
            end
        else
            interNear.Visible = 'off';
        end

        if midInterCheck == 1
            interMid.Visible = 'on';
            [mMid, ~] = size(interMidPoly.Vertices);
            if (mMid < 3)
                interMid.Shape.Vertices = [];
            else
                interMid.Shape.Vertices = interMidPoly.Vertices;
            end
        else
            interMid.Visible = 'off';
        end

        if farInterCheck == 1
            interFar.Visible = 'on';
            [mFar, ~] = size(interFarPoly.Vertices);
            if (mFar < 3)
                interFar.Shape.Vertices = [];
            else
                interFar.Shape.Vertices = interFarPoly.Vertices;
            end
        else
            interFar.Visible = 'off';
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
        
        % Построение объектов (препятствий) на графике
        for j = 1:maxNumberOfObjects
            paralSurf{j,1}.Vertices = parallelepipeds{j,1}.Points;
            hold on;
            if j > numberOfObjects
                paralSurf{j,1}.Visible = 'off';
            else
                paralSurf{j,1}.Visible = 'on';
            end
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
        
        switch numberOfObjects
            case 0
                set(bObjectX1,'Visible','off');
                set(bObjectY1,'Visible','off');
                set(bObjectZ1,'Visible','off');
                set(bObjectWidth1,'Visible','off');
                set(bObjectDepth1,'Visible','off');
                set(bObjectHeight1,'Visible','off');
                set(bObjectX2,'Visible','off');
                set(bObjectY2,'Visible','off');
                set(bObjectZ2,'Visible','off');
                set(bObjectWidth2,'Visible','off');
                set(bObjectDepth2,'Visible','off');
                set(bObjectHeight2,'Visible','off');
                set(bObjectX3,'Visible','off');
                set(bObjectY3,'Visible','off');
                set(bObjectZ3,'Visible','off');
                set(bObjectWidth3,'Visible','off');
                set(bObjectDepth3,'Visible','off');
                set(bObjectHeight3,'Visible','off');
                set(bObjectX4,'Visible','off');
                set(bObjectY4,'Visible','off');
                set(bObjectZ4,'Visible','off');
                set(bObjectWidth4,'Visible','off');
                set(bObjectDepth4,'Visible','off');
                set(bObjectHeight4,'Visible','off');
                set(bObjectX5,'Visible','off');
                set(bObjectY5,'Visible','off');
                set(bObjectZ5,'Visible','off');
                set(bObjectWidth5,'Visible','off');
                set(bObjectDepth5,'Visible','off');
                set(bObjectHeight5,'Visible','off');

                set(bObjectX1T,'Visible','off');
                set(bObjectY1T,'Visible','off');
                set(bObjectZ1T,'Visible','off');
                set(bObjectWidth1T,'Visible','off');
                set(bObjectDepth1T,'Visible','off');
                set(bObjectHeight1T,'Visible','off');
                set(bObjectX2T,'Visible','off');
                set(bObjectY2T,'Visible','off');
                set(bObjectZ2T,'Visible','off');
                set(bObjectWidth2T,'Visible','off');
                set(bObjectDepth2T,'Visible','off');
                set(bObjectHeight2T,'Visible','off');
                set(bObjectX3T,'Visible','off');
                set(bObjectY3T,'Visible','off');
                set(bObjectZ3T,'Visible','off');
                set(bObjectWidth3T,'Visible','off');
                set(bObjectDepth3T,'Visible','off');
                set(bObjectHeight3T,'Visible','off');
                set(bObjectX4T,'Visible','off');
                set(bObjectY4T,'Visible','off');
                set(bObjectZ4T,'Visible','off');
                set(bObjectWidth4T,'Visible','off');
                set(bObjectDepth4T,'Visible','off');
                set(bObjectHeight4T,'Visible','off');
                set(bObjectX5T,'Visible','off');
                set(bObjectY5T,'Visible','off');
                set(bObjectZ5T,'Visible','off');
                set(bObjectWidth5T,'Visible','off');
                set(bObjectDepth5T,'Visible','off');
                set(bObjectHeight5T,'Visible','off');
            case 1
                set(bObjectX1,'Visible','on');
                set(bObjectY1,'Visible','on');
                set(bObjectZ1,'Visible','on');
                set(bObjectWidth1,'Visible','on');
                set(bObjectDepth1,'Visible','on');
                set(bObjectHeight1,'Visible','on');
                set(bObjectX2,'Visible','off');
                set(bObjectY2,'Visible','off');
                set(bObjectZ2,'Visible','off');
                set(bObjectWidth2,'Visible','off');
                set(bObjectDepth2,'Visible','off');
                set(bObjectHeight2,'Visible','off');
                set(bObjectX3,'Visible','off');
                set(bObjectY3,'Visible','off');
                set(bObjectZ3,'Visible','off');
                set(bObjectWidth3,'Visible','off');
                set(bObjectDepth3,'Visible','off');
                set(bObjectHeight3,'Visible','off');
                set(bObjectX4,'Visible','off');
                set(bObjectY4,'Visible','off');
                set(bObjectZ4,'Visible','off');
                set(bObjectWidth4,'Visible','off');
                set(bObjectDepth4,'Visible','off');
                set(bObjectHeight4,'Visible','off');
                set(bObjectX5,'Visible','off');
                set(bObjectY5,'Visible','off');
                set(bObjectZ5,'Visible','off');
                set(bObjectWidth5,'Visible','off');
                set(bObjectDepth5,'Visible','off');
                set(bObjectHeight5,'Visible','off');

                set(bObjectX1T,'Visible','on');
                set(bObjectY1T,'Visible','on');
                set(bObjectZ1T,'Visible','on');
                set(bObjectWidth1T,'Visible','on');
                set(bObjectDepth1T,'Visible','on');
                set(bObjectHeight1T,'Visible','on');
                set(bObjectX2T,'Visible','off');
                set(bObjectY2T,'Visible','off');
                set(bObjectZ2T,'Visible','off');
                set(bObjectWidth2T,'Visible','off');
                set(bObjectDepth2T,'Visible','off');
                set(bObjectHeight2T,'Visible','off');
                set(bObjectX3T,'Visible','off');
                set(bObjectY3T,'Visible','off');
                set(bObjectZ3T,'Visible','off');
                set(bObjectWidth3T,'Visible','off');
                set(bObjectDepth3T,'Visible','off');
                set(bObjectHeight3T,'Visible','off');
                set(bObjectX4T,'Visible','off');
                set(bObjectY4T,'Visible','off');
                set(bObjectZ4T,'Visible','off');
                set(bObjectWidth4T,'Visible','off');
                set(bObjectDepth4T,'Visible','off');
                set(bObjectHeight4T,'Visible','off');
                set(bObjectX5T,'Visible','off');
                set(bObjectY5T,'Visible','off');
                set(bObjectZ5T,'Visible','off');
                set(bObjectWidth5T,'Visible','off');
                set(bObjectDepth5T,'Visible','off');
                set(bObjectHeight5T,'Visible','off');
            case 2
                set(bObjectX1,'Visible','on');
                set(bObjectY1,'Visible','on');
                set(bObjectZ1,'Visible','on');
                set(bObjectWidth1,'Visible','on');
                set(bObjectDepth1,'Visible','on');
                set(bObjectHeight1,'Visible','on');
                set(bObjectX2,'Visible','on');
                set(bObjectY2,'Visible','on');
                set(bObjectZ2,'Visible','on');
                set(bObjectWidth2,'Visible','on');
                set(bObjectDepth2,'Visible','on');
                set(bObjectHeight2,'Visible','on');
                set(bObjectX3,'Visible','off');
                set(bObjectY3,'Visible','off');
                set(bObjectZ3,'Visible','off');
                set(bObjectWidth3,'Visible','off');
                set(bObjectDepth3,'Visible','off');
                set(bObjectHeight3,'Visible','off');
                set(bObjectX4,'Visible','off');
                set(bObjectY4,'Visible','off');
                set(bObjectZ4,'Visible','off');
                set(bObjectWidth4,'Visible','off');
                set(bObjectDepth4,'Visible','off');
                set(bObjectHeight4,'Visible','off');
                set(bObjectX5,'Visible','off');
                set(bObjectY5,'Visible','off');
                set(bObjectZ5,'Visible','off');
                set(bObjectWidth5,'Visible','off');
                set(bObjectDepth5,'Visible','off');
                set(bObjectHeight5,'Visible','off');

                set(bObjectX1T,'Visible','on');
                set(bObjectY1T,'Visible','on');
                set(bObjectZ1T,'Visible','on');
                set(bObjectWidth1T,'Visible','on');
                set(bObjectDepth1T,'Visible','on');
                set(bObjectHeight1T,'Visible','on');
                set(bObjectX2T,'Visible','on');
                set(bObjectY2T,'Visible','on');
                set(bObjectZ2T,'Visible','on');
                set(bObjectWidth2T,'Visible','on');
                set(bObjectDepth2T,'Visible','on');
                set(bObjectHeight2T,'Visible','on');
                set(bObjectX3T,'Visible','off');
                set(bObjectY3T,'Visible','off');
                set(bObjectZ3T,'Visible','off');
                set(bObjectWidth3T,'Visible','off');
                set(bObjectDepth3T,'Visible','off');
                set(bObjectHeight3T,'Visible','off');
                set(bObjectX4T,'Visible','off');
                set(bObjectY4T,'Visible','off');
                set(bObjectZ4T,'Visible','off');
                set(bObjectWidth4T,'Visible','off');
                set(bObjectDepth4T,'Visible','off');
                set(bObjectHeight4T,'Visible','off');
                set(bObjectX5T,'Visible','off');
                set(bObjectY5T,'Visible','off');
                set(bObjectZ5T,'Visible','off');
                set(bObjectWidth5T,'Visible','off');
                set(bObjectDepth5T,'Visible','off');
                set(bObjectHeight5T,'Visible','off');
            case 3
                set(bObjectX1,'Visible','on');
                set(bObjectY1,'Visible','on');
                set(bObjectZ1,'Visible','on');
                set(bObjectWidth1,'Visible','on');
                set(bObjectDepth1,'Visible','on');
                set(bObjectHeight1,'Visible','on');
                set(bObjectX2,'Visible','on');
                set(bObjectY2,'Visible','on');
                set(bObjectZ2,'Visible','on');
                set(bObjectWidth2,'Visible','on');
                set(bObjectDepth2,'Visible','on');
                set(bObjectHeight2,'Visible','on');
                set(bObjectX3,'Visible','on');
                set(bObjectY3,'Visible','on');
                set(bObjectZ3,'Visible','on');
                set(bObjectWidth3,'Visible','on');
                set(bObjectDepth3,'Visible','on');
                set(bObjectHeight3,'Visible','on');
                set(bObjectX4,'Visible','off');
                set(bObjectY4,'Visible','off');
                set(bObjectZ4,'Visible','off');
                set(bObjectWidth4,'Visible','off');
                set(bObjectDepth4,'Visible','off');
                set(bObjectHeight4,'Visible','off');
                set(bObjectX5,'Visible','off');
                set(bObjectY5,'Visible','off');
                set(bObjectZ5,'Visible','off');
                set(bObjectWidth5,'Visible','off');
                set(bObjectDepth5,'Visible','off');
                set(bObjectHeight5,'Visible','off');

                set(bObjectX1T,'Visible','on');
                set(bObjectY1T,'Visible','on');
                set(bObjectZ1T,'Visible','on');
                set(bObjectWidth1T,'Visible','on');
                set(bObjectDepth1T,'Visible','on');
                set(bObjectHeight1T,'Visible','on');
                set(bObjectX2T,'Visible','on');
                set(bObjectY2T,'Visible','on');
                set(bObjectZ2T,'Visible','on');
                set(bObjectWidth2T,'Visible','on');
                set(bObjectDepth2T,'Visible','on');
                set(bObjectHeight2T,'Visible','on');
                set(bObjectX3T,'Visible','on');
                set(bObjectY3T,'Visible','on');
                set(bObjectZ3T,'Visible','on');
                set(bObjectWidth3T,'Visible','on');
                set(bObjectDepth3T,'Visible','on');
                set(bObjectHeight3T,'Visible','on');
                set(bObjectX4T,'Visible','off');
                set(bObjectY4T,'Visible','off');
                set(bObjectZ4T,'Visible','off');
                set(bObjectWidth4T,'Visible','off');
                set(bObjectDepth4T,'Visible','off');
                set(bObjectHeight4T,'Visible','off');
                set(bObjectX5T,'Visible','off');
                set(bObjectY5T,'Visible','off');
                set(bObjectZ5T,'Visible','off');
                set(bObjectWidth5T,'Visible','off');
                set(bObjectDepth5T,'Visible','off');
                set(bObjectHeight5T,'Visible','off');
            case 4
                set(bObjectX1,'Visible','on');
                set(bObjectY1,'Visible','on');
                set(bObjectZ1,'Visible','on');
                set(bObjectWidth1,'Visible','on');
                set(bObjectDepth1,'Visible','on');
                set(bObjectHeight1,'Visible','on');
                set(bObjectX2,'Visible','on');
                set(bObjectY2,'Visible','on');
                set(bObjectZ2,'Visible','on');
                set(bObjectWidth2,'Visible','on');
                set(bObjectDepth2,'Visible','on');
                set(bObjectHeight2,'Visible','on');
                set(bObjectX3,'Visible','on');
                set(bObjectY3,'Visible','on');
                set(bObjectZ3,'Visible','on');
                set(bObjectWidth3,'Visible','on');
                set(bObjectDepth3,'Visible','on');
                set(bObjectHeight3,'Visible','on');
                set(bObjectX4,'Visible','on');
                set(bObjectY4,'Visible','on');
                set(bObjectZ4,'Visible','on');
                set(bObjectWidth4,'Visible','on');
                set(bObjectDepth4,'Visible','on');
                set(bObjectHeight4,'Visible','on');
                set(bObjectX5,'Visible','off');
                set(bObjectY5,'Visible','off');
                set(bObjectZ5,'Visible','off');
                set(bObjectWidth5,'Visible','off');
                set(bObjectDepth5,'Visible','off');
                set(bObjectHeight5,'Visible','off');

                set(bObjectX1T,'Visible','on');
                set(bObjectY1T,'Visible','on');
                set(bObjectZ1T,'Visible','on');
                set(bObjectWidth1T,'Visible','on');
                set(bObjectDepth1T,'Visible','on');
                set(bObjectHeight1T,'Visible','on');
                set(bObjectX2T,'Visible','on');
                set(bObjectY2T,'Visible','on');
                set(bObjectZ2T,'Visible','on');
                set(bObjectWidth2T,'Visible','on');
                set(bObjectDepth2T,'Visible','on');
                set(bObjectHeight2T,'Visible','on');
                set(bObjectX3T,'Visible','on');
                set(bObjectY3T,'Visible','on');
                set(bObjectZ3T,'Visible','on');
                set(bObjectWidth3T,'Visible','on');
                set(bObjectDepth3T,'Visible','on');
                set(bObjectHeight3T,'Visible','on');
                set(bObjectX4T,'Visible','on');
                set(bObjectY4T,'Visible','on');
                set(bObjectZ4T,'Visible','on');
                set(bObjectWidth4T,'Visible','on');
                set(bObjectDepth4T,'Visible','on');
                set(bObjectHeight4T,'Visible','on');
                set(bObjectX5T,'Visible','off');
                set(bObjectY5T,'Visible','off');
                set(bObjectZ5T,'Visible','off');
                set(bObjectWidth5T,'Visible','off');
                set(bObjectDepth5T,'Visible','off');
                set(bObjectHeight5T,'Visible','off');
            case 5
                set(bObjectX1,'Visible','on');
                set(bObjectY1,'Visible','on');
                set(bObjectZ1,'Visible','on');
                set(bObjectWidth1,'Visible','on');
                set(bObjectDepth1,'Visible','on');
                set(bObjectHeight1,'Visible','on');
                set(bObjectX2,'Visible','on');
                set(bObjectY2,'Visible','on');
                set(bObjectZ2,'Visible','on');
                set(bObjectWidth2,'Visible','on');
                set(bObjectDepth2,'Visible','on');
                set(bObjectHeight2,'Visible','on');
                set(bObjectX3,'Visible','on');
                set(bObjectY3,'Visible','on');
                set(bObjectZ3,'Visible','on');
                set(bObjectWidth3,'Visible','on');
                set(bObjectDepth3,'Visible','on');
                set(bObjectHeight3,'Visible','on');
                set(bObjectX4,'Visible','on');
                set(bObjectY4,'Visible','on');
                set(bObjectZ4,'Visible','on');
                set(bObjectWidth4,'Visible','on');
                set(bObjectDepth4,'Visible','on');
                set(bObjectHeight4,'Visible','on');
                set(bObjectX5,'Visible','on');
                set(bObjectY5,'Visible','on');
                set(bObjectZ5,'Visible','on');
                set(bObjectWidth5,'Visible','on');
                set(bObjectDepth5,'Visible','on');
                set(bObjectHeight5,'Visible','on');

                set(bObjectX1T,'Visible','on');
                set(bObjectY1T,'Visible','on');
                set(bObjectZ1T,'Visible','on');
                set(bObjectWidth1T,'Visible','on');
                set(bObjectDepth1T,'Visible','on');
                set(bObjectHeight1T,'Visible','on');
                set(bObjectX2T,'Visible','on');
                set(bObjectY2T,'Visible','on');
                set(bObjectZ2T,'Visible','on');
                set(bObjectWidth2T,'Visible','on');
                set(bObjectDepth2T,'Visible','on');
                set(bObjectHeight2T,'Visible','on');
                set(bObjectX3T,'Visible','on');
                set(bObjectY3T,'Visible','on');
                set(bObjectZ3T,'Visible','on');
                set(bObjectWidth3T,'Visible','on');
                set(bObjectDepth3T,'Visible','on');
                set(bObjectHeight3T,'Visible','on');
                set(bObjectX4T,'Visible','on');
                set(bObjectY4T,'Visible','on');
                set(bObjectZ4T,'Visible','on');
                set(bObjectWidth4T,'Visible','on');
                set(bObjectDepth4T,'Visible','on');
                set(bObjectHeight4T,'Visible','on');
                set(bObjectX5T,'Visible','on');
                set(bObjectY5T,'Visible','on');
                set(bObjectZ5T,'Visible','on');
                set(bObjectWidth5T,'Visible','on');
                set(bObjectDepth5T,'Visible','on');
                set(bObjectHeight5T,'Visible','on');
        end
    end
end