function plotFrustumIntersect(W,H,pan,tilt,roll,fovH,fovV,...
    farClipPlane,camPos,heightLimit,numberOfObjects,...
    roomW,roomD,roomH,camW,camD,camH)

    identPPM = 250;
    recogPPM = 125;
    visibPPM = 62;
    detectPPM = 25;
    monitorPPM = 12;
    
    fovHTan = tan(fovH / 2 / 180 * pi);
    fovVTan = tan(fovV / 2 / 180 * pi);
    
    aspect = W / H;
    
    if (fovHTan <= 0)
        fovHTan = fovVTan * aspect;
    end
    
    if (fovVTan <= 0)
        fovVTan = fovHTan / aspect;
    end

    R = findRotationMatrix(pan,tilt,roll);      % Матрица поворота
    X = [0 0 1];                                % Вектор нормали (также просто ненулевой вектор)
    T = X * R;                                  % Вектор направления камеры
    
    identDist = W / (2 * identPPM * fovHTan);       % Расстояние идентификации
    recogDist = W / (2 * recogPPM * fovHTan);       % Расстояние распознавания
    visibDist = W / (2 * visibPPM * fovHTan);       % Расстояние обзора
    detectDist = W / (2 * detectPPM * fovHTan);     % Расстояние детекции
    monitorDist = W / (2 * monitorPPM * fovHTan);   % Расстояние мониторинга
    
    if (identDist > farClipPlane)
        identDist = farClipPlane;
        recogDist = farClipPlane;
        visibDist = farClipPlane;
        detectDist = farClipPlane;
        monitorDist = farClipPlane;
    elseif (recogDist > farClipPlane)
        recogDist = farClipPlane;
        visibDist = farClipPlane;
        detectDist = farClipPlane;
        monitorDist = farClipPlane;
    elseif (visibDist > farClipPlane)
        visibDist = farClipPlane;
        detectDist = farClipPlane;
        monitorDist = farClipPlane;
    elseif (detectDist > farClipPlane)
        detectDist = farClipPlane;
        monitorDist = farClipPlane;
    elseif (monitorDist > farClipPlane)
        monitorDist = farClipPlane;
    end
    
    identCenter = camPos + T * identDist;       % Центр основания frustum'a идентификации
    recogCenter = camPos + T * recogDist;       % Центр основания frustum'a распознования
    visibCenter = camPos + T * visibDist;       % Центр основания frustum'a обзора
    detectCenter = camPos + T * detectDist;     % Центр основания frustum'a детекции
    monitorCenter = camPos + T * monitorDist;   % Центр основания frustum'a мониторинга
    fcpCenter = camPos + T * farClipPlane;      % Центр основания дальнего frustum

    % Координаты точек основания frustum
    [upRightIdent,upLeftIdent,downRightIdent,downLeftIdent] = findFrustumBase(identCenter,fovHTan,fovVTan,R,identDist);
    [upRightRecog,upLeftRecog,downRightRecog,downLeftRecog] = findFrustumBase(recogCenter,fovHTan,fovVTan,R,recogDist);
    [upRightVisib,upLeftVisib,downRightVisib,downLeftVisib] = findFrustumBase(visibCenter,fovHTan,fovVTan,R,visibDist);
    [upRightDetect,upLeftDetect,downRightDetect,downLeftDetect] = findFrustumBase(detectCenter,fovHTan,fovVTan,R,detectDist);
    [upRightMonitor,upLeftMonitor,downRightMonitor,downLeftMonitor] = findFrustumBase(monitorCenter,fovHTan,fovVTan,R,monitorDist);
    [upRightFar,upLeftFar,downRightFar,downLeftFar] = findFrustumBase(fcpCenter,fovHTan,fovVTan,R,farClipPlane);
    
    V0 = [0 0 0];                                   % Любая точка на плоскости пола
    room = getParallelepiped(roomW,roomD,roomH,V0); % Триангулированное помещение

    % Нахождение точек пересечения frustum'ов с плоскостью пола
    planeInterIdent = planeFrustumIntersect(X,V0,camPos,upRightIdent,upLeftIdent,downRightIdent,downLeftIdent);
    planeInterRecog = planeTruncFrustumIntersect(X,V0,upRightIdent,upLeftIdent,downRightIdent,downLeftIdent,...
        upRightRecog,upLeftRecog,downRightRecog,downLeftRecog);
    planeInterVisib = planeTruncFrustumIntersect(X,V0,upRightRecog,upLeftRecog,downRightRecog,downLeftRecog,...
        upRightVisib,upLeftVisib,downRightVisib,downLeftVisib);
    planeInterDetect = planeTruncFrustumIntersect(X,V0,upRightVisib,upLeftVisib,downRightVisib,downLeftVisib,...
        upRightDetect,upLeftDetect,downRightDetect,downLeftDetect);
    planeInterMonitor = planeTruncFrustumIntersect(X,V0,upRightDetect,upLeftDetect,downRightDetect,downLeftDetect,...
        upRightMonitor,upLeftMonitor,downRightMonitor,downLeftMonitor);
    planeInterFar = planeTruncFrustumIntersect(X,V0,upRightMonitor,upLeftMonitor,downRightMonitor,downLeftMonitor,...
        upRightFar,upLeftFar,downRightFar,downLeftFar);
    
    % Определение плоскости ограничения по высоте
    V1 = [0 1 heightLimit];                 % Любая точка на плоскости пола
    X1 = [0 0 heightLimit + 1];             % Вектор нормали плоскости
    
    % Нахождение точек пересечения полного frustum с плоскостью ограничения
    % по высоте
    planeInterLimit = planeFrustumIntersect(X1,V1,camPos,upRightMonitor,upLeftMonitor,downRightMonitor,downLeftMonitor);
    
    % Нахождение точек пересечения полного frustum с плоскостью пола
    planeInterFloor = planeFrustumIntersect(X,V0,camPos,upRightMonitor,upLeftMonitor,downRightMonitor,downLeftMonitor);
    
    % Нахождение конъюнкции многоугольников на высоте ограничения и на полу
    conjunction = planesConjunction(planeInterLimit,planeInterFloor);
    
    % Создание объектов и нахождение слепых зон
    interIdentPoly = polyshape(planeInterIdent(:,1:2));
    interRecogPoly = polyshape(planeInterRecog(:,1:2));
    interVisibPoly = polyshape(planeInterVisib(:,1:2));
    interDetectPoly = polyshape(planeInterDetect(:,1:2));
    interMonitorPoly = polyshape(planeInterMonitor(:,1:2));
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
        interIdentPoly = intersect(interIdentPoly,floorPoly);
        interRecogPoly = intersect(interRecogPoly,floorPoly);
        interVisibPoly = intersect(interVisibPoly,floorPoly);
        interDetectPoly = intersect(interDetectPoly,floorPoly);
        interMonitorPoly = intersect(interMonitorPoly,floorPoly);
        interFarPoly = intersect(interFarPoly,floorPoly);
    end
    
    % Определение доступного места для установки камеры
    roomAvailableW = roomW - 2 * camW;
    roomAvailableD = roomD - 2 * camW;
    roomAvailableH = roomH - (2 * camH + heightLimit);
    
    pointsNumberW = ceil(roomAvailableW * 2);
    pointsNumberD = ceil(roomAvailableD * 2);
    pointsNumberH = ceil(roomAvailableH * 2);
    
    placementStepW = roomAvailableW / pointsNumberW;
    placementStepD = roomAvailableD / pointsNumberD;
    placementStepH = roomAvailableH / pointsNumberH;
    
    % Расположение сетки точек возможных положений камеры (без учёта окон и
    % дверей)
    pointsX = zeros(pointsNumberW,1);
    pointsY = zeros(pointsNumberD,1);
    pointsZ = zeros(pointsNumberH,1);
    
    gridWallNear = cell(pointsNumberH,1);
    gridWallLeft = cell(pointsNumberH,1);
    gridWallFar = cell(pointsNumberH,1);
    gridWallRight = cell(pointsNumberH,1);
    gridRoof = cell(pointsNumberD,1);
    
    for i = 1:pointsNumberW         % Общие координаты точек сетки по X
        pointsX(i,1) = camW + placementStepW * i;
    end
    
    for i = 1:pointsNumberD         % Общие координаты точек сетки по Y
        pointsY(i,1) = camW + placementStepD * i;
    end
    
    for i = 1:pointsNumberH         % Общие координаты точек сетки по Z
        pointsZ(i,1) = camH + heightLimit + placementStepH * i;
    end
    
    for i = 1:pointsNumberH         % Заполнение сетки точками для ближней стены
        gridWallNear{i,1} = [pointsX zeros(pointsNumberW,1) ones(pointsNumberW,1) * pointsZ(i,1)];
    end
    gridCam.WallNear = cat(1,gridWallNear{:});
    
    for i = 1:pointsNumberH         % Заполнение сетки точками для левой стены
        gridWallLeft{i,1} = [zeros(pointsNumberD,1) pointsY ones(pointsNumberD,1) * pointsZ(i,1)];
    end
    gridCam.WallLeft = cat(1,gridWallLeft{:});
    
    for i = 1:pointsNumberH         % Заполнение сетки точками для дальней стены
        gridWallFar{i,1} = [pointsX ones(pointsNumberW,1) * roomD ones(pointsNumberW,1) * pointsZ(i,1)];
    end
    gridCam.WallFar = cat(1,gridWallFar{:});

    for i = 1:pointsNumberH         % Заполнение сетки точками для правой стены
        gridWallRight{i,1} = [ones(pointsNumberD,1) * roomW pointsY ones(pointsNumberD,1) * pointsZ(i,1)];
    end
    gridCam.WallRight = cat(1,gridWallRight{:});
    
    for i = 1:pointsNumberD         % Заполнение сетки точками для потолка
        gridRoof{i,1} = [pointsX ones(pointsNumberW,1) * pointsY(i,1) ones(pointsNumberW,1) * roomH];
    end
    gridCam.Roof = cat(1,gridRoof{:});
    
    f = figure;
    identColor = 'red';
    recogColor = 'yellow';
    visibColor = 'green';
    detectColor = 'cyan';
    monitorColor = 'blue';
    fColor = 'white';
    conColor = 'black';
    paralColor = 'magenta';
    gridColor = 'green';

    % Если рёбра frustum пересекают плоскость пола, то строим пересечение на
    % графике (вместе со слепыми зонами)
    [mIdent, ~] = size(interIdentPoly.Vertices);
    if (mIdent < 3)
        interIdent = plot(polyshape([0 0 0 0], [0 0 0 0]),'FaceColor',identColor);
        hold on
    else
        interIdent = plot(interIdentPoly,'FaceColor',identColor);
        hold on
    end
    
    [mRecog, ~] = size(interRecogPoly.Vertices);
    if (mRecog < 3)
        interRecog = plot(polyshape([0 0 0 0], [0 0 0 0]),'FaceColor',recogColor);
        hold on
    else
        interRecog = plot(interRecogPoly,'FaceColor',recogColor);
        hold on
    end
    
    [mVisib, ~] = size(interVisibPoly.Vertices);
    if (mVisib < 3)
        interVisib = plot(polyshape([0 0 0 0], [0 0 0 0]),'FaceColor',visibColor);
        hold on
    else
        interVisib = plot(interVisibPoly,'FaceColor',visibColor);
        hold on
    end
    
    [mDetect, ~] = size(interDetectPoly.Vertices);
    if (mDetect < 3)
        interDetect = plot(polyshape([0 0 0 0], [0 0 0 0]),'FaceColor',detectColor);
        hold on
    else
        interDetect = plot(interDetectPoly,'FaceColor',detectColor);
        hold on
    end
    
    [mMonitor, ~] = size(interMonitorPoly.Vertices);
    if (mMonitor < 3)
        interMonitor = plot(polyshape([0 0 0 0], [0 0 0 0]),'FaceColor',monitorColor);
        hold on
    else
        interMonitor = plot(interMonitorPoly,'FaceColor',monitorColor);
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
    PIdent = [upRightIdent; upLeftIdent; downLeftIdent; downRightIdent; camPos];
    indUpF = [1 2 5]; frustumUpIdent = fill3(PIdent(indUpF, 1), PIdent(indUpF, 2), PIdent(indUpF, 3), identColor);
    indLeftF = [2 3 5]; frustumLeftIdent = fill3(PIdent(indLeftF, 1), PIdent(indLeftF, 2), PIdent(indLeftF, 3), identColor);
    indDownF = [3 4 5]; frustumDownIdent = fill3(PIdent(indDownF, 1), PIdent(indDownF, 2), PIdent(indDownF, 3), identColor);
    indRightF = [4 1 5]; frustumRightIdent = fill3(PIdent(indRightF, 1), PIdent(indRightF, 2), PIdent(indRightF, 3), identColor);
    hold on
    
    indUp = [1 2 6 5];
    indLeft = [2 3 7 6]; 
    indDown = [3 4 8 7]; 
    indRight = [4 1 5 8]; 
    
    PRecog = [upRightIdent; upLeftIdent; downLeftIdent; downRightIdent; ...
        upRightRecog; upLeftRecog; downLeftRecog; downRightRecog];
    frustumUpRecog = fill3(PRecog(indUp, 1), PRecog(indUp, 2), PRecog(indUp, 3), recogColor);
    frustumLeftRecog = fill3(PRecog(indLeft, 1), PRecog(indLeft, 2), PRecog(indLeft, 3), recogColor);
    frustumDownRecog = fill3(PRecog(indDown, 1), PRecog(indDown, 2), PRecog(indDown, 3), recogColor);
    frustumRightRecog = fill3(PRecog(indRight, 1), PRecog(indRight, 2), PRecog(indRight, 3), recogColor);
    hold on
    
    PVisib = [upRightRecog; upLeftRecog; downLeftRecog; downRightRecog; ...
        upRightVisib; upLeftVisib; downLeftVisib; downRightVisib];
    frustumUpVisib = fill3(PVisib(indUp, 1), PVisib(indUp, 2), PVisib(indUp, 3), visibColor);
    frustumLeftVisib = fill3(PVisib(indLeft, 1), PVisib(indLeft, 2), PVisib(indLeft, 3), visibColor);
    frustumDownVisib = fill3(PVisib(indDown, 1), PVisib(indDown, 2), PVisib(indDown, 3), visibColor);
    frustumRightVisib = fill3(PVisib(indRight, 1), PVisib(indRight, 2), PVisib(indRight, 3), visibColor);
    hold on
    
    PDetect = [upRightVisib; upLeftVisib; downLeftVisib; downRightVisib; ...
        upRightDetect; upLeftDetect; downLeftDetect; downRightDetect];
    frustumUpDetect = fill3(PDetect(indUp, 1), PDetect(indUp, 2), PDetect(indUp, 3), detectColor);
    frustumLeftDetect = fill3(PDetect(indLeft, 1), PDetect(indLeft, 2), PDetect(indLeft, 3), detectColor);
    frustumDownDetect = fill3(PDetect(indDown, 1), PDetect(indDown, 2), PDetect(indDown, 3), detectColor);
    frustumRightDetect = fill3(PDetect(indRight, 1), PDetect(indRight, 2), PDetect(indRight, 3), detectColor);
    hold on
    
    PMonitor = [upRightDetect; upLeftDetect; downLeftDetect; downRightDetect; ...
        upRightMonitor; upLeftMonitor; downLeftMonitor; downRightMonitor];
    frustumUpMonitor = fill3(PMonitor(indUp, 1), PMonitor(indUp, 2), PMonitor(indUp, 3), monitorColor);
    frustumLeftMonitor = fill3(PMonitor(indLeft, 1), PMonitor(indLeft, 2), PMonitor(indLeft, 3), monitorColor);
    frustumDownMonitor = fill3(PMonitor(indDown, 1), PMonitor(indDown, 2), PMonitor(indDown, 3), monitorColor);
    frustumRightMonitor = fill3(PMonitor(indRight, 1), PMonitor(indRight, 2), PMonitor(indRight, 3), monitorColor);
    hold on
    
    PFar = [upRightMonitor; upLeftMonitor; downLeftMonitor; downRightMonitor; ...
        upRightFar; upLeftFar; downLeftFar; downRightFar];
    frustumUpFar = fill3(PFar(indUp, 1), PFar(indUp, 2), PFar(indUp, 3), fColor);
    frustumLeftFar = fill3(PFar(indLeft, 1), PFar(indLeft, 2), PFar(indLeft, 3), fColor);
    frustumDownFar = fill3(PFar(indDown, 1), PFar(indDown, 2), PFar(indDown, 3), fColor);
    frustumRightFar = fill3(PFar(indRight, 1), PFar(indRight, 2), PFar(indRight, 3), fColor);
    hold on
    
    % Построение помещения
    roomSurf = trisurf(room,'FaceColor','none','LineWidth',2);
    alpha(0.2);
    hold on;
    
    % Построение сетки на стенах и потолке
    gridWallNearPlot = plot3(gridCam.WallNear(:,1),gridCam.WallNear(:,2),gridCam.WallNear(:,3),'.','Color',gridColor);
    gridWallLeftPlot = plot3(gridCam.WallLeft(:,1),gridCam.WallLeft(:,2),gridCam.WallLeft(:,3),'.','Color',gridColor);
    gridWallFarPlot = plot3(gridCam.WallFar(:,1),gridCam.WallFar(:,2),gridCam.WallFar(:,3),'.','Color',gridColor);
    gridWallRightPlot = plot3(gridCam.WallRight(:,1),gridCam.WallRight(:,2),gridCam.WallRight(:,3),'.','Color',gridColor);
    gridRoofPlot = plot3(gridCam.Roof(:,1),gridCam.Roof(:,2),gridCam.Roof(:,3),'.','Color',gridColor);

    % Построение камеры
    pose = rigid3d(R,camPos);
    cam = plotCamera('AbsolutePose',pose,'Opacity',0,'AxesVisible',false,'Size',camD);

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
    
    bWidth = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*2+distH,54,50,23],...
                  'Value', W, 'String', W, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81*2+distH,25,50,23],...
                    'String','Width','BackgroundColor',bgcolor);
                
    bHeight = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*2+distH,54+distV,50,23],...
                  'Value', H, 'String', H, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81*2+distH,25+distV,50,23],...
                    'String','Height','BackgroundColor',bgcolor);
    
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
                
    bFrustumIdent = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH,54-distV/2,130,23],...
                   'Value', 1, 'String', 'Ident Frustum', 'Callback', {@update3DPointS});
               
    bFrustumRecog = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH,54,130,23],...
                   'Value', 1, 'String', 'Recog Frustum', 'Callback', {@update3DPointS});

    bFrustumVisib = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH,54+distV/2,130,23],...
                   'Value', 1, 'String', 'Visible Frustum', 'Callback', {@update3DPointS});
    
    bFrustumDetect = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH,54+distV,130,23],...
                   'Value', 1, 'String', 'Detect Frustum', 'Callback', {@update3DPointS});
               
    bFrustumMonitor = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH,54+distV*3/2,130,23],...
                   'Value', 1, 'String', 'Monitor Frustum', 'Callback', {@update3DPointS});
               
    bFrustumFar = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH,54+distV*2,130,23],...
                   'Value', 1, 'String', 'Far Frustum', 'Callback', {@update3DPointS});
    
    bInterIdent = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81,54+distV*5/2,130,23],...
                   'Value', 1, 'String', 'Ident Intersection', 'Callback', {@update3DPointS});
               
    bInterRecog = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81,54+distV*3,130,23],...
                   'Value', 1, 'String', 'Recog Intersection', 'Callback', {@update3DPointS});

    bInterVisib = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81,54+distV*7/2,130,23],...
                   'Value', 1, 'String', 'Visib Intersection', 'Callback', {@update3DPointS});           
    
    bInterDetect = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81,54+distV*4,130,23],...
                   'Value', 1, 'String', 'Detect Intersection', 'Callback', {@update3DPointS});
               
    bInterMonitor = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81,54+distV*9/2,130,23],...
                   'Value', 1, 'String', 'Monitor Intersection', 'Callback', {@update3DPointS});           
               
    bInterFar = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81,54+distV*5,130,23],...
                   'Value', 1, 'String', 'Far Intersection', 'Callback', {@update3DPointS});
    
    bInterHeight = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH+161,54-distV/2,130,23],...
                   'Value', 1, 'String', 'Height Intersection', 'Callback', {@update3DPointS});
               
    bInterCon = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH+161,54,130,23],...
                   'Value', 1, 'String', 'Conjunction Intersection', 'Callback', {@update3DPointS});
               
    bRoomW = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81+130,54+distV*3,50,23],...
                  'Value', roomW, 'String', roomW, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81+130,25+distV*3,50,23],...
                    'String','Room Width','BackgroundColor',bgcolor);

    bRoomD = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81+130,54+distV*4,50,23],...
                  'Value', roomD, 'String', roomD, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81+130,25+distV*4,50,23],...
                    'String','Room Depth','BackgroundColor',bgcolor);
    
    bRoomH = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81+130,54+distV*5,50,23],...
                  'Value', roomH, 'String', roomH, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81+130,25+distV*5,50,23],...
                    'String','Room Height','BackgroundColor',bgcolor);
                
    bCamW = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*2+130,54+distV*3,50,23],...
                  'Value', camW, 'String', camW, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81*2+130,25+distV*3,50,23],...
                    'String','Camera Width','BackgroundColor',bgcolor);

    bCamD = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*2+130,54+distV*4,50,23],...
                  'Value', camD, 'String', camD, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81*2+130,25+distV*4,50,23],...
                    'String','Camera Depth','BackgroundColor',bgcolor);
    
    bCamH = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*2+130,54+distV*5,50,23],...
                  'Value', camH, 'String', camH, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81*2+130,25+distV*5,50,23],...
                    'String','Camera Height','BackgroundColor',bgcolor);
    
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
        W = str2double(get(bWidth,'String'));
        H = str2double(get(bHeight,'String'));
        farClipPlane = str2double(get(bFarClipPlane,'String'));
        fovH = str2double(get(bFovH,'String'));
        fovV = str2double(get(bFovV,'String'));
        heightLimit = str2double(get(bHeightLimit,'String'));
        identFrustumCheck = get(bFrustumIdent,'Value');
        recogFrustumCheck = get(bFrustumRecog,'Value');
        visibFrustumCheck = get(bFrustumVisib,'Value');
        detectFrustumCheck = get(bFrustumDetect,'Value');
        monitorFrustumCheck = get(bFrustumMonitor,'Value');
        farFrustumCheck = get(bFrustumFar,'Value');
        identInterCheck = get(bInterIdent,'Value');
        recogInterCheck = get(bInterRecog,'Value');
        visibInterCheck = get(bInterVisib,'Value');
        detectInterCheck = get(bInterDetect,'Value');
        monitorInterCheck = get(bInterMonitor,'Value');
        farInterCheck = get(bInterFar,'Value');
        heightInterCheck = get(bInterHeight,'Value');
        conInterCheck = get(bInterCon,'Value');
        numberOfObjects = get(bNumberOfObjects,'Value') - 1;
        roomW = str2double(get(bRoomW,'String'));
        roomD = str2double(get(bRoomD,'String'));
        roomH = str2double(get(bRoomH,'String'));
        camW = str2double(get(bCamW,'String'));
        camD = str2double(get(bCamD,'String'));
        camH = str2double(get(bCamH,'String'));
        
        camPos = [camPosX camPosY camPosZ];

        fovHTan = tan(fovH / 2 / 180 * pi);
        fovVTan = tan(fovV / 2 / 180 * pi);
        
        aspect = W / H;
        
        if fovHTan <= 0
            fovHTan = fovVTan * aspect;
        end

        if fovVTan <= 0
            fovVTan = fovHTan / aspect;
        end
        
        R = findRotationMatrix(pan,tilt,roll);
        pose = rigid3d(R,camPos);
        cam.AbsolutePose = pose;
        cam.Size = camD;

        X = [0 0 1];                                % Вектор нормали (также просто ненулевой вектор)
        T = X * R;                                  % Вектор направления камеры
        
        identDist = W / (2 * identPPM * fovHTan);       % Расстояние идентификации
        recogDist = W / (2 * recogPPM * fovHTan);       % Расстояние распознавания
        visibDist = W / (2 * visibPPM * fovHTan);       % Расстояние обзора
        detectDist = W / (2 * detectPPM * fovHTan);     % Расстояние детекции
        monitorDist = W / (2 * monitorPPM * fovHTan);   % Расстояние мониторинга

        if (identDist > farClipPlane)
            identDist = farClipPlane;
            recogDist = farClipPlane;
            visibDist = farClipPlane;
            detectDist = farClipPlane;
            monitorDist = farClipPlane;
        elseif (recogDist > farClipPlane)
            recogDist = farClipPlane;
            visibDist = farClipPlane;
            detectDist = farClipPlane;
            monitorDist = farClipPlane;
        elseif (visibDist > farClipPlane)
            visibDist = farClipPlane;
            detectDist = farClipPlane;
            monitorDist = farClipPlane;
        elseif (detectDist > farClipPlane)
            detectDist = farClipPlane;
            monitorDist = farClipPlane;
        elseif (monitorDist > farClipPlane)
            monitorDist = farClipPlane;
        end

        identCenter = camPos + T * identDist;       % Центр основания frustum'a идентификации
        recogCenter = camPos + T * recogDist;       % Центр основания frustum'a распознования
        visibCenter = camPos + T * visibDist;       % Центр основания frustum'a обзора
        detectCenter = camPos + T * detectDist;     % Центр основания frustum'a детекции
        monitorCenter = camPos + T * monitorDist;   % Центр основания frustum'a мониторинга
        fcpCenter = camPos + T * farClipPlane;      % Центр основания дальнего frustum

        % Координаты точек основания frustum
        [upRightIdent,upLeftIdent,downRightIdent,downLeftIdent] = findFrustumBase(identCenter,fovHTan,fovVTan,R,identDist);
        [upRightRecog,upLeftRecog,downRightRecog,downLeftRecog] = findFrustumBase(recogCenter,fovHTan,fovVTan,R,recogDist);
        [upRightVisib,upLeftVisib,downRightVisib,downLeftVisib] = findFrustumBase(visibCenter,fovHTan,fovVTan,R,visibDist);
        [upRightDetect,upLeftDetect,downRightDetect,downLeftDetect] = findFrustumBase(detectCenter,fovHTan,fovVTan,R,detectDist);
        [upRightMonitor,upLeftMonitor,downRightMonitor,downLeftMonitor] = findFrustumBase(monitorCenter,fovHTan,fovVTan,R,monitorDist);
        [upRightFar,upLeftFar,downRightFar,downLeftFar] = findFrustumBase(fcpCenter,fovHTan,fovVTan,R,farClipPlane);
        
        room = getParallelepiped(roomW,roomD,roomH,V0); % Триангулированное помещение
        
        % Нахождение точек пересечения frustum с плоскостью пола
        planeInterIdent = planeFrustumIntersect(X,V0,camPos,upRightIdent,upLeftIdent,downRightIdent,downLeftIdent);
        planeInterRecog = planeTruncFrustumIntersect(X,V0,upRightIdent,upLeftIdent,downRightIdent,downLeftIdent,...
            upRightRecog,upLeftRecog,downRightRecog,downLeftRecog);
        planeInterVisib = planeTruncFrustumIntersect(X,V0,upRightRecog,upLeftRecog,downRightRecog,downLeftRecog,...
            upRightVisib,upLeftVisib,downRightVisib,downLeftVisib);
        planeInterDetect = planeTruncFrustumIntersect(X,V0,upRightVisib,upLeftVisib,downRightVisib,downLeftVisib,...
            upRightDetect,upLeftDetect,downRightDetect,downLeftDetect);
        planeInterMonitor = planeTruncFrustumIntersect(X,V0,upRightDetect,upLeftDetect,downRightDetect,downLeftDetect,...
            upRightMonitor,upLeftMonitor,downRightMonitor,downLeftMonitor);
        planeInterFar = planeTruncFrustumIntersect(X,V0,upRightMonitor,upLeftMonitor,downRightMonitor,downLeftMonitor,...
            upRightFar,upLeftFar,downRightFar,downLeftFar);
        
        % Определение плоскости ограничения по высоте
        V1 = [0 1 heightLimit];                 % Любая точка на плоскости пола
        X1 = [0 0 heightLimit + 1];             % Вектор нормали плоскости
        
        % Нахождение точек пересечения полного frustum с плоскостью ограничения
        % по высоте
        planeInterLimit = planeFrustumIntersect(X1,V1,camPos,upRightMonitor,upLeftMonitor,downRightMonitor,downLeftMonitor);

        % Нахождение точек пересечения полного frustum с плоскостью пола
        planeInterFloor = planeFrustumIntersect(X,V0,camPos,upRightMonitor,upLeftMonitor,downRightMonitor,downLeftMonitor);

        % Нахождения конъюнкции многоугольников на высоте ограничения и на полу
        conjunction = planesConjunction(planeInterLimit,planeInterFloor);
        
        % Создание объектов и нахождение слепых зон
        interIdentPoly = polyshape(planeInterIdent(:,1:2));
        interRecogPoly = polyshape(planeInterRecog(:,1:2));
        interVisibPoly = polyshape(planeInterVisib(:,1:2));
        interDetectPoly = polyshape(planeInterDetect(:,1:2));
        interMonitorPoly = polyshape(planeInterMonitor(:,1:2));
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
            interIdentPoly = intersect(interIdentPoly,floorPoly);
            interRecogPoly = intersect(interRecogPoly,floorPoly);
            interVisibPoly = intersect(interVisibPoly,floorPoly);
            interDetectPoly = intersect(interDetectPoly,floorPoly);
            interMonitorPoly = intersect(interMonitorPoly,floorPoly);
            interFarPoly = intersect(interFarPoly,floorPoly);
        end
        
        % Определение доступного места для установки камеры
        roomAvailableW = roomW - 2 * camW;
        roomAvailableD = roomD - 2 * camW;
        roomAvailableH = roomH - (2 * camH + heightLimit);

        pointsNumberW = ceil(roomAvailableW * 2);
        pointsNumberD = ceil(roomAvailableD * 2);
        pointsNumberH = ceil(roomAvailableH * 2);

        placementStepW = roomAvailableW / pointsNumberW;
        placementStepD = roomAvailableD / pointsNumberD;
        placementStepH = roomAvailableH / pointsNumberH;

        % Расположение сетки точек возможных положений камеры (без учёта окон и
        % дверей)
        pointsX = zeros(pointsNumberW,1);
        pointsY = zeros(pointsNumberD,1);
        pointsZ = zeros(pointsNumberH,1);

        gridWallNear = cell(pointsNumberH,1);
        gridWallLeft = cell(pointsNumberH,1);
        gridWallFar = cell(pointsNumberH,1);
        gridWallRight = cell(pointsNumberH,1);
        gridRoof = cell(pointsNumberD,1);

        for j = 1:pointsNumberW         % Общие координаты точек сетки по X
            pointsX(j,1) = camW + placementStepW * j;
        end

        for j = 1:pointsNumberD         % Общие координаты точек сетки по Y
            pointsY(j,1) = camW + placementStepD * j;
        end

        for j = 1:pointsNumberH         % Общие координаты точек сетки по Z
            pointsZ(j,1) = camH + heightLimit + placementStepH * j;
        end

        for j = 1:pointsNumberH         % Заполнение сетки точками для ближней стены
            gridWallNear{j,1} = [pointsX zeros(pointsNumberW,1) ones(pointsNumberW,1) * pointsZ(j,1)];
        end
        gridCam.WallNear = cat(1,gridWallNear{:});

        for j = 1:pointsNumberH         % Заполнение сетки точками для левой стены
            gridWallLeft{j,1} = [zeros(pointsNumberD,1) pointsY ones(pointsNumberD,1) * pointsZ(j,1)];
        end
        gridCam.WallLeft = cat(1,gridWallLeft{:});

        for j = 1:pointsNumberH         % Заполнение сетки точками для дальней стены
            gridWallFar{j,1} = [pointsX ones(pointsNumberW,1) * roomD ones(pointsNumberW,1) * pointsZ(j,1)];
        end
        gridCam.WallFar = cat(1,gridWallFar{:});

        for j = 1:pointsNumberH         % Заполнение сетки точками для правой стены
            gridWallRight{j,1} = [ones(pointsNumberD,1) * roomW pointsY ones(pointsNumberD,1) * pointsZ(j,1)];
        end
        gridCam.WallRight = cat(1,gridWallRight{:});

        for j = 1:pointsNumberD         % Заполнение сетки точками для потолка
            gridRoof{j,1} = [pointsX ones(pointsNumberW,1) * pointsY(j,1) ones(pointsNumberW,1) * roomH];
        end
        gridCam.Roof = cat(1,gridRoof{:});
        
        % Перемещение polyshape на нужную высоту (на будущее)
        %     M = [   1   0   0   0
        %             0   1   0   0
        %             0   0   1   10
        %             0   0   0   1   ];
        %     t=hgtransform('Matrix',M);     
        %     Hpgon = plot(interFarPoly,'Parent',t,'FaceColor','r');

        % Если рёбра frustum пересекают плоскость пола, то строим пересечение на
        % графике
        if identInterCheck == 1
            interIdent.Visible = 'on';
            [mIdent, ~] = size(interIdentPoly.Vertices);
            if (mIdent < 3)
                interIdent.Shape.Vertices = [];
            else
                interIdent.Shape.Vertices = interIdentPoly.Vertices;
            end
        else
            interIdent.Visible = 'off';
        end

        if recogInterCheck == 1
            interRecog.Visible = 'on';
            [mRecog, ~] = size(interRecogPoly.Vertices);
            if (mRecog < 3)
                interRecog.Shape.Vertices = [];
            else
                interRecog.Shape.Vertices = interRecogPoly.Vertices;
            end
        else
            interRecog.Visible = 'off';
        end
        
        if visibInterCheck == 1
            interVisib.Visible = 'on';
            [mVisib, ~] = size(interVisibPoly.Vertices);
            if (mVisib < 3)
                interVisib.Shape.Vertices = [];
            else
                interVisib.Shape.Vertices = interVisibPoly.Vertices;
            end
        else
            interVisib.Visible = 'off';
        end
        
        if detectInterCheck == 1
            interDetect.Visible = 'on';
            [mDetect, ~] = size(interDetectPoly.Vertices);
            if (mDetect < 3)
                interDetect.Shape.Vertices = [];
            else
                interDetect.Shape.Vertices = interDetectPoly.Vertices;
            end
        else
            interDetect.Visible = 'off';
        end
        
        if monitorInterCheck == 1
            interMonitor.Visible = 'on';
            [mMonitor, ~] = size(interMonitorPoly.Vertices);
            if (mMonitor < 3)
                interMonitor.Shape.Vertices = [];
            else
                interMonitor.Shape.Vertices = interMonitorPoly.Vertices;
            end
        else
            interMonitor.Visible = 'off';
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
            set(interHeight,'visible','on');
            [mHeight, ~] = size(planeInterLimit);
            if (mHeight < 3)
                set(interHeight,'XData',[0 0 0],'YData',[0 0 0],'ZData',[0 0 0]);
            else
                set(interHeight,'XData',planeInterLimit(:,1),'YData',planeInterLimit(:,2),'ZData',planeInterLimit(:,3));
            end
        else
            set(interHeight,'visible','off');
        end

        % Если конъюнкция плоскостей - многоугольник, то строим её на графике
        if conInterCheck == 1
            set(interCon,'visible','on');
            [mCon, ~] = size(conjunction);
            if (mCon < 3)
                set(interCon,'XData',[0 0 0],'YData',[0 0 0],'ZData',[0 0 0]);
            else
                set(interCon,'XData',conjunction(:,1),'YData',conjunction(:,2),'ZData',conjunction(:,3));
            end
        else
            set(interCon,'visible','off');
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
        
        % Построение помещения
        roomSurf.Vertices = room.Points;
        hold on;

        % Построение сетки на стенах и потолке
        set(gridWallNearPlot,'XData',gridCam.WallNear(:,1),'YData',gridCam.WallNear(:,2),'ZData',gridCam.WallNear(:,3));
        set(gridWallLeftPlot,'XData',gridCam.WallLeft(:,1),'YData',gridCam.WallLeft(:,2),'ZData',gridCam.WallLeft(:,3));
        set(gridWallFarPlot,'XData',gridCam.WallFar(:,1),'YData',gridCam.WallFar(:,2),'ZData',gridCam.WallFar(:,3));
        set(gridWallRightPlot,'XData',gridCam.WallRight(:,1),'YData',gridCam.WallRight(:,2),'ZData',gridCam.WallRight(:,3));
        set(gridRoofPlot,'XData',gridCam.Roof(:,1),'YData',gridCam.Roof(:,2),'ZData',gridCam.Roof(:,3));

        % Переопределение построенных frustum`ов
        if identFrustumCheck == 1
            set(frustumUpIdent, 'visible', 'on');
            set(frustumLeftIdent, 'visible', 'on');
            set(frustumDownIdent, 'visible', 'on');
            set(frustumRightIdent, 'visible', 'on');
            PIdent = [upRightIdent; upLeftIdent; downLeftIdent; downRightIdent; camPos];
            set(frustumUpIdent, 'XData', PIdent(indUpF, 1), 'YData', PIdent(indUpF, 2), 'ZData', PIdent(indUpF, 3));
            set(frustumLeftIdent, 'XData', PIdent(indLeftF, 1), 'YData', PIdent(indLeftF, 2), 'ZData', PIdent(indLeftF, 3));
            set(frustumDownIdent, 'XData', PIdent(indDownF, 1), 'YData', PIdent(indDownF, 2), 'ZData', PIdent(indDownF, 3));
            set(frustumRightIdent, 'XData', PIdent(indRightF, 1), 'YData', PIdent(indRightF, 2), 'ZData', PIdent(indRightF, 3));
        else
            set(frustumUpIdent, 'visible', 'off');
            set(frustumLeftIdent, 'visible', 'off');
            set(frustumDownIdent, 'visible', 'off');
            set(frustumRightIdent, 'visible', 'off');
        end
        
        if recogFrustumCheck == 1
            set(frustumUpRecog, 'visible', 'on');
            set(frustumLeftRecog, 'visible', 'on');
            set(frustumDownRecog, 'visible', 'on');
            set(frustumRightRecog, 'visible', 'on');
            PRecog = [upRightIdent; upLeftIdent; downLeftIdent; downRightIdent; ...
                upRightRecog; upLeftRecog; downLeftRecog; downRightRecog];
            set(frustumUpRecog, 'XData', PRecog(indUp, 1), 'YData', PRecog(indUp, 2), 'ZData', PRecog(indUp, 3));
            set(frustumLeftRecog, 'XData', PRecog(indLeft, 1), 'YData', PRecog(indLeft, 2), 'ZData', PRecog(indLeft, 3));
            set(frustumDownRecog, 'XData', PRecog(indDown, 1), 'YData', PRecog(indDown, 2), 'ZData', PRecog(indDown, 3));
            set(frustumRightRecog, 'XData', PRecog(indRight, 1), 'YData', PRecog(indRight, 2), 'ZData', PRecog(indRight, 3));
        else
            set(frustumUpRecog, 'visible', 'off');
            set(frustumLeftRecog, 'visible', 'off');
            set(frustumDownRecog, 'visible', 'off');
            set(frustumRightRecog, 'visible', 'off');
        end
        
        if visibFrustumCheck == 1
            set(frustumUpVisib, 'visible', 'on');
            set(frustumLeftVisib, 'visible', 'on');
            set(frustumDownVisib, 'visible', 'on');
            set(frustumRightVisib, 'visible', 'on');
            PVisib = [upRightRecog; upLeftRecog; downLeftRecog; downRightRecog; ...
                upRightVisib; upLeftVisib; downLeftVisib; downRightVisib];
            set(frustumUpVisib, 'XData', PVisib(indUp, 1), 'YData', PVisib(indUp, 2), 'ZData', PVisib(indUp, 3));
            set(frustumLeftVisib, 'XData', PVisib(indLeft, 1), 'YData', PVisib(indLeft, 2), 'ZData', PVisib(indLeft, 3));
            set(frustumDownVisib, 'XData', PVisib(indDown, 1), 'YData', PVisib(indDown, 2), 'ZData', PVisib(indDown, 3));
            set(frustumRightVisib, 'XData', PVisib(indRight, 1), 'YData', PVisib(indRight, 2), 'ZData', PVisib(indRight, 3));
        else
            set(frustumUpVisib, 'visible', 'off');
            set(frustumLeftVisib, 'visible', 'off');
            set(frustumDownVisib, 'visible', 'off');
            set(frustumRightVisib, 'visible', 'off');
        end
        
        if detectFrustumCheck == 1
            set(frustumUpDetect, 'visible', 'on');
            set(frustumLeftDetect, 'visible', 'on');
            set(frustumDownDetect, 'visible', 'on');
            set(frustumRightDetect, 'visible', 'on');
            PDetect = [upRightVisib; upLeftVisib; downLeftVisib; downRightVisib; ...
                upRightDetect; upLeftDetect; downLeftDetect; downRightDetect];
            set(frustumUpDetect, 'XData', PDetect(indUp, 1), 'YData', PDetect(indUp, 2), 'ZData', PDetect(indUp, 3));
            set(frustumLeftDetect, 'XData', PDetect(indLeft, 1), 'YData', PDetect(indLeft, 2), 'ZData', PDetect(indLeft, 3));
            set(frustumDownDetect, 'XData', PDetect(indDown, 1), 'YData', PDetect(indDown, 2), 'ZData', PDetect(indDown, 3));
            set(frustumRightDetect, 'XData', PDetect(indRight, 1), 'YData', PDetect(indRight, 2), 'ZData', PDetect(indRight, 3));
        else
            set(frustumUpDetect, 'visible', 'off');
            set(frustumLeftDetect, 'visible', 'off');
            set(frustumDownDetect, 'visible', 'off');
            set(frustumRightDetect, 'visible', 'off');
        end
        
        if monitorFrustumCheck == 1
            set(frustumUpMonitor, 'visible', 'on');
            set(frustumLeftMonitor, 'visible', 'on');
            set(frustumDownMonitor, 'visible', 'on');
            set(frustumRightMonitor, 'visible', 'on');
            PMonitor = [upRightDetect; upLeftDetect; downLeftDetect; downRightDetect; ...
                upRightMonitor; upLeftMonitor; downLeftMonitor; downRightMonitor];
            set(frustumUpMonitor, 'XData', PMonitor(indUp, 1), 'YData', PMonitor(indUp, 2), 'ZData', PMonitor(indUp, 3));
            set(frustumLeftMonitor, 'XData', PMonitor(indLeft, 1), 'YData', PMonitor(indLeft, 2), 'ZData', PMonitor(indLeft, 3));
            set(frustumDownMonitor, 'XData', PMonitor(indDown, 1), 'YData', PMonitor(indDown, 2), 'ZData', PMonitor(indDown, 3));
            set(frustumRightMonitor, 'XData', PMonitor(indRight, 1), 'YData', PMonitor(indRight, 2), 'ZData', PMonitor(indRight, 3));
        else
            set(frustumUpMonitor, 'visible', 'off');
            set(frustumLeftMonitor, 'visible', 'off');
            set(frustumDownMonitor, 'visible', 'off');
            set(frustumRightMonitor, 'visible', 'off');
        end

        if farFrustumCheck == 1
            set(frustumUpFar, 'visible', 'on');
            set(frustumLeftFar, 'visible', 'on');
            set(frustumDownFar, 'visible', 'on');
            set(frustumRightFar, 'visible', 'on');
            PFar = [upRightMonitor; upLeftMonitor; downLeftMonitor; downRightMonitor; ...
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