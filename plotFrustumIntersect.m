function plotFrustumIntersect(W,H,pan,tilt,roll,fovH,fovV,...
    farClipPlane,camPos,heightLimit,heightLimitIdent,numberOfObjects,...
    wallsPts,roomH,gridStep,camW,camD,camH,nearClipPlaneDist)

    identPPM = 250;
    recogPPM = 125;
    visibPPM = 62;
    detectPPM = 25;
    monitorPPM = 12;
    
    aspect = W / H;
        
    if fovH <= 0
        fovH = fovV * aspect;
    end

    if fovV <= 0
        fovV = fovH / aspect;
    end
    
    fovHTan = tan(fovH / 2 / 180 * pi);
    fovVTan = tan(fovV / 2 / 180 * pi);

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
    [upRightFar,upLeftFar,~,~] = findFrustumBase(fcpCenter,fovHTan,fovVTan,R,farClipPlane);
    
    V0 = [0 0 0];                               % Любая точка на плоскости пола
    [wallsCount,~] = size(wallsPts);
    room = cell(wallsCount + 1,1);              % Создание помещения
    roomSize = zeros(wallsCount,1);
    roofPtsOrder = zeros(wallsCount,2);
    for i = 1:wallsCount
        if i ~= wallsCount
            wall = getWall(wallsPts(i,:),wallsPts(i + 1,:),roomH);
            wallW = norm(wallsPts(i,:) - wallsPts(i + 1,:));
            
            roofPtsOrder(i,1) = i;
            roofPtsOrder(i,2) = i + 1;
        else
            wall = getWall(wallsPts(i,:),wallsPts(1,:),roomH);
            wallW = norm(wallsPts(i,:) - wallsPts(1,:));
            
            roofPtsOrder(i,1) = i;
            roofPtsOrder(i,2) = 1;
        end
        room{i,1} = wall;
        roomSize(i,1) = wallW;
    end
    
    DT = delaunayTriangulation(wallsPts,roofPtsOrder);      % Возможно, лишняя триангуляция
    isInside = isInterior(DT);      % Поиск треугольников внутри ограниченных рёбер
    tri = DT(isInside, :);          % Получить индексы точек внутренних треугольников
    roof = triangulation(tri,[wallsPts ones(wallsCount,1) * roomH]);
    room{wallsCount + 1,1} = roof;

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
    
    % Определение плоскости ограничения по высоте
    V1 = [0 1 heightLimit];                 % Любая точка на плоскости пола
    X1 = [0 0 heightLimit + 1];             % Вектор нормали плоскости
    
    % Определение плоскости допустимого ограничения по высоте для идентификации
    V2 = [0 1 heightLimitIdent];            % Любая точка на плоскости пола
    X2 = [0 0 heightLimitIdent + 1];        % Вектор нормали плоскости
    
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
    
    maxNumberOfObjects = 5;
    parallelepipeds = cell(maxNumberOfObjects,1);
    
    floorPoly = polyshape(planeInterFloor(:,1:2));
    nearClipPlane = camPos + T * nearClipPlaneDist;
    
    X3 = X2;
    V3 = V2;

    for i = 1:maxNumberOfObjects
        parallelepipeds{i,1} = getParallelepiped(i,i,i*2,[i*2 i*2 0]);
        if i <= numberOfObjects 
            if isBehind(parallelepipeds{i,1},T,nearClipPlane,'full')
                continue
            elseif isBehind(parallelepipeds{i,1},T,nearClipPlane,'any')
                    TRInterCamera = planeObjectIntersection(parallelepipeds{i,1},T,nearClipPlane);
                if (isBehind(TRInterCamera,X3,V3,'full'))
                    continue
                elseif (isBehind(TRInterCamera,X3,V3,'any'))
                    TRInter = planeObjectIntersection(TRInterCamera,X3,V3);
                    TRPlane = planeProjection(TRInter,X3,V3,T,camPos,upRightFar,upLeftFar);
                else
                    TRPlane = planeProjection(TRInterCamera,X3,V3,T,camPos,upRightFar,upLeftFar);
                end
                unPoly = unionPolygons(TRPlane);
                floorPoly = subtract(floorPoly,unPoly);
            else
                if (isBehind(parallelepipeds{i,1},X3,V3,'full'))
                    continue
                elseif (isBehind(parallelepipeds{i,1},X3,V3,'any'))
                    TRInter = planeObjectIntersection(parallelepipeds{i,1},X3,V3);
                    TRPlane = planeProjection(TRInter,X3,V3,T,camPos,upRightFar,upLeftFar);
                else
                    TRPlane = planeProjection(parallelepipeds{i,1},X3,V3,T,camPos,upRightFar,upLeftFar);
                end
                unPoly = unionPolygons(TRPlane);
                floorPoly = subtract(floorPoly,unPoly);
            end
        end
    end

    for i = 1:wallsCount
        if isBehind(room{i,1},T,nearClipPlane,'full')
            continue
        elseif isBehind(room{i,1},T,nearClipPlane,'any')
            TRInterCamera = planeObjectIntersection(room{i,1},T,nearClipPlane);
            if (isBehind(room{i,1},X,V0,'full'))
                continue
            elseif (isBehind(room{i,1},X,V0,'any'))
                TRInter = planeObjectIntersection(TRInterCamera,X,V0);
                TRPlane = planeProjection(TRInter,X,V0,T,camPos,upRightFar,upLeftFar);
            else
                TRPlane = planeProjection(TRInterCamera,X,V0,T,camPos,upRightFar,upLeftFar);
            end
            unPoly = unionPolygons(TRPlane);
            floorPoly = subtract(floorPoly,unPoly);
        else
            if (isBehind(room{i,1},X,V0,'full'))
                continue
            elseif (isBehind(room{i,1},X,V0,'any'))
                TRInter = planeObjectIntersection(room{i,1},X,V0);
                TRPlane = planeProjection(TRInter,X,V0,T,camPos,upRightFar,upLeftFar);
            else
                TRPlane = planeProjection(room{i,1},X,V0,T,camPos,upRightFar,upLeftFar);
            end
            unPoly = unionPolygons(TRPlane);
            floorPoly = subtract(floorPoly,unPoly);
        end
    end
    
    % Удаление зоны видимости камеры снаружи помещения (не костыль)
    floorRoom = polyshape(wallsPts);
    floorPoly = intersect(floorPoly,floorRoom);
    floorPoly = intersect(floorPoly,conjunction);   % Пересечение с ограничением по высоте
   
    interIdentPoly = intersect(interIdentPoly,floorPoly);
    interRecogPoly = intersect(interRecogPoly,floorPoly);
    interVisibPoly = intersect(interVisibPoly,floorPoly);
    interDetectPoly = intersect(interDetectPoly,floorPoly);
    interMonitorPoly = intersect(interMonitorPoly,floorPoly);
    
    % Определение доступного места для установки камеры и вычисление сетки
    % точек возможных положений камеры (без учёта окон и дверей)
    gridWalls = cell(wallsCount,1);
    wallAvailableH = roomH - (2 * camH + heightLimit);
    pointsNumberH = floor(wallAvailableH / gridStep);
    
    pointsH = zeros(pointsNumberH,1);
    for i = 1:pointsNumberH         % Общие координаты точек сетки по Z
        pointsH(i,1) = camH + heightLimit + gridStep * i;
    end
    
    % Расчёт сетки точек для стен
    for i = 1:wallsCount
        wallW = roomSize(i,1);                              % Ширина стены
        wallAvailableW = wallW - 2 * camW;                  % Доступная часть стены
        pointsNumberW = floor(wallAvailableW / gridStep);   % Количество точек сетки
        
        startPoint = wallsPts(roofPtsOrder(i,1),:);         % Начальная координата стены
        endPoint = wallsPts(roofPtsOrder(i,2),:);           % Конечная координата стены
        direction = endPoint - startPoint;                  % Вектор стены
        directionScaled = direction/norm(direction);        % Вектор длиной 1
        camWScaled = startPoint + directionScaled * camW;   % Начало сетки по ширине с учётом габаритов камеры
        
        pointsW = zeros(pointsNumberW,2);                   % Точки сетки по ширине
        for k = 1:pointsNumberW
            pointsW(k,:) = camWScaled + directionScaled * gridStep * k;
        end
        
        gridWall = cell(pointsNumberH,1);                   % Точки сетки по высоте
        for k = 1:pointsNumberH
            gridWall{k,1} = [pointsW ones(pointsNumberW,1) * pointsH(k,1)];
        end
        
        gridWalls{i,1} = cat(1,gridWall{:});
    end
    
    % Расчёт сетки точек для потолка
    [rectX,rectY] = minBoundRect(wallsPts(:,1),wallsPts(:,2));  % Описание помещения прямоугольником
    roofStartPointW = [rectX(1,1) rectY(1,1)];
    roofStartPointD = [rectX(2,1) rectY(2,1)];
    roofEndPointD = [rectX(3,1) rectY(3,1)];
    roofW = norm(roofStartPointD - roofStartPointW);
    roofD = norm(roofEndPointD - roofStartPointD);
    
    roofAvailableW = roofW - 2 * camW;
    roofAvailableD = roofD - 2 * camW;
    roofPointsNumberW = floor(roofAvailableW / gridStep);
    roofPointsNumberD = floor(roofAvailableD / gridStep);
    
    roofDirectionW = roofStartPointD - roofStartPointW;     % Вектор одной стороны описанного прямоугольника 
    roofDirectionD = roofEndPointD - roofStartPointD;       % Вектор другой стороны описанного прямоугольника 
    roofDirectionWScaled = roofDirectionW/norm(roofDirectionW);     % Вектора длиной 1
    roofDirectionDScaled = roofDirectionD/norm(roofDirectionD);
    roofCamWScaled = roofStartPointW + roofDirectionWScaled * camW;
    
    roofPointsW = zeros(roofPointsNumberW,2);
    gridRoof = zeros(roofPointsNumberW * roofPointsNumberD,2);
    
    for i = 1:roofPointsNumberW
        roofPointsW(i,:) = roofCamWScaled + roofDirectionWScaled * gridStep * i;
    end
    
    for i = 1:roofPointsNumberW
        roofCamDScaled = roofPointsW(i,:) + roofDirectionDScaled * camW;
        for k = 1:roofPointsNumberD
            gridRoof((i - 1) * roofPointsNumberD + k,:) = roofCamDScaled + roofDirectionDScaled * gridStep * k;
        end
    end
    
    inRoof = inpolygon(gridRoof(:,1),gridRoof(:,2),...      % Удаление точек, которые не попадают на потолок
        [wallsPts(:,1); wallsPts(1,1)],[wallsPts(:,2); wallsPts(1,2)]);
    for i = roofPointsNumberW * roofPointsNumberD:-1:1
        if ~inRoof(i) || cameraWallIntersect(gridRoof(i,1:2),wallsPts,camW)
            gridRoof(i,:) = [];
        end
    end
    
    [mGridRoof,~] = size(gridRoof);
    gridRoof = [gridRoof ones(mGridRoof,1) * roomH];
    
    f = figure;
    identColor = 'red';
    recogColor = 'yellow';
    visibColor = 'green';
    detectColor = 'cyan';
    monitorColor = 'blue';
    conColor = 'black';
    paralColor = 'magenta';
    gridColor = [0 0.4470 0.7410];
    paintRoomColor = 'green';

    % Если рёбра frustum пересекают плоскость пола, то строим пересечение на
    % графике (вместе со слепыми зонами)
    [mIdent, ~] = size(interIdentPoly.Vertices);
    if (mIdent < 3)
        interIdent = plot(polyshape(),'FaceColor',identColor);
        hold on
    else
        interIdent = plot(interIdentPoly,'FaceColor',identColor);
        hold on
    end
    
    [mRecog, ~] = size(interRecogPoly.Vertices);
    if (mRecog < 3)
        interRecog = plot(polyshape(),'FaceColor',recogColor);
        hold on
    else
        interRecog = plot(interRecogPoly,'FaceColor',recogColor);
        hold on
    end
    
    [mVisib, ~] = size(interVisibPoly.Vertices);
    if (mVisib < 3)
        interVisib = plot(polyshape(),'FaceColor',visibColor);
        hold on
    else
        interVisib = plot(interVisibPoly,'FaceColor',visibColor);
        hold on
    end
    
    [mDetect, ~] = size(interDetectPoly.Vertices);
    if (mDetect < 3)
        interDetect = plot(polyshape(),'FaceColor',detectColor);
        hold on
    else
        interDetect = plot(interDetectPoly,'FaceColor',detectColor);
        hold on
    end
    
    [mMonitor, ~] = size(interMonitorPoly.Vertices);
    if (mMonitor < 3)
        interMonitor = plot(polyshape(),'FaceColor',monitorColor);
        hold on
    else
        interMonitor = plot(interMonitorPoly,'FaceColor',monitorColor);
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
        set(interHeight,'visible','off');
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
    
    % Построение помещения
    roomPlot = cell(wallsCount + 1,1);
    quiverPlot = cell(wallsCount + 1,1);
    for i = 1:wallsCount+1
        roomPlot{i,1} = trisurf(room{i,1},'FaceColor',paintRoomColor,'LineWidth',2);
        
        P1 = incenter(room{i,1});
        F1 = faceNormal(room{i,1});
        quiverPlot{i,1} = quiver3(P1(:,1),P1(:,2),P1(:,3), ...
            F1(:,1),F1(:,2),F1(:,3),0.5,'color','r');
        set(quiverPlot{i,1},'Visible','off');
        hold on;
    end
    
    % Построение сетки на стенах и потолке
    gridWallsPlot = cell(wallsCount,1);
    for i = 1:wallsCount
        gridWallsPlot{i,1} = plot3(gridWalls{i,1}(:,1),gridWalls{i,1}(:,2),gridWalls{i,1}(:,3),'.','Color',gridColor);
        hold on;
    end
    
    gridRoofPlot = plot3(gridRoof(:,1),gridRoof(:,2),gridRoof(:,3),'.','Color',gridColor);
    hold on;
    alpha(0.2);

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
                  'Value', pan, 'min', 0, 'max', 360, 'SliderStep', [1/360 5/360],... 
                  'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[50,54,23,23],...
                    'String','0','BackgroundColor',bgcolor);
    uicontrol('Parent',f,'Style','text','Position',[500,54,23,23],...
                    'String','360','BackgroundColor',bgcolor);
    bPanText = uicontrol('Parent',f,'Style','text','Position',[240,25,100,23],...
                    'String',"Pan " + pan,'BackgroundColor',bgcolor);
              
    bTilt = uicontrol('Parent', f, 'Style', 'slider', 'Position', [81,54+distV,419,23],...
                  'Value', tilt, 'min', 0, 'max', 90, 'SliderStep', [1/90 5/90],...
                  'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[50,54+distV,23,23],...
                    'String','0','BackgroundColor',bgcolor);
    uicontrol('Parent',f,'Style','text','Position',[500,54+distV,23,23],...
                    'String','90','BackgroundColor',bgcolor);
    bTiltText = uicontrol('Parent',f,'Style','text','Position',[240,25+distV,100,23],...
                    'String',"Tilt " + tilt,'BackgroundColor',bgcolor); 
                
    bRoll = uicontrol('Parent', f, 'Style', 'slider', 'Position', [81,54+distV*2,419,23],...
                  'Value', roll, 'min', 0, 'max', 360, 'SliderStep', [1/360 5/360],...
                  'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[50,54+distV*2,23,23],...
                    'String','0','BackgroundColor',bgcolor);
    uicontrol('Parent',f,'Style','text','Position',[500,54+distV*2,23,23],...
                    'String','360','BackgroundColor',bgcolor);
    bRollText = uicontrol('Parent',f,'Style','text','Position',[240,25+distV*2,100,23],...
                    'String',"Roll (unused) " + roll,'BackgroundColor',bgcolor);
                
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
                
    bLimitIdent = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81*4+distH,54+distV*2,50,23],...
                  'Value', heightLimitIdent, 'String', heightLimitIdent, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81*4+distH,25+distV*2,50,23],...
                    'String','Height Limit Ident','BackgroundColor',bgcolor);
                
    bFrustum = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH,54-distV/2,130,23],...
                   'Value', 1, 'String', 'Frustum', 'Callback', {@update3DPointS});
               
    bInter = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH,54,130,23],...
                   'Value', 1, 'String', 'Visibility Area', 'Callback', {@update3DPointS});
               
    bBestLocation = uicontrol('Parent', f, 'Style', 'pushbutton', 'Position', [81*4+distH,54+distV/2,130,23],...
                   'String', 'Best Camera Location', 'Callback', {@bestCameraLocation});
               
    bSimpleAlgorithm = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH,54+distV,130,23],...
                   'Value', 1, 'String', 'Simplified Algorithm');
    
    bInterHeight = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH+161,54-distV/2,130,23],...
                   'Value', 0, 'String', 'Height Intersection', 'Callback', {@update3DPointS});
               
    bHeightLimitInter = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH+161,54,130,23],...
                   'Value', 1, 'String', 'Height Limit', 'Callback', {@update3DPointS});
               
    bInterLimitIdent = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH+161,54+distV/2,130,23],...
                   'Value', 1, 'String', 'Identity Height Limit', 'Callback', {@update3DPointS});
               
    bNormals = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81*4+distH+161,54+distV,130,23],...
                   'Value', 0, 'String', 'Normals', 'Callback', {@update3DPointS});
    
    bGridRoom = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81,54+distV*5/2,130,23],...
                   'Value', 1, 'String', 'Grid Room', 'Callback', {@update3DPointS});
               
    bCutRoom = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81,54+distV*3,130,23],...
                   'Value', 0, 'String', 'Cut Room', 'Callback', {@update3DPointS});
               
    bPaintRoom = uicontrol('Parent', f, 'Style', 'checkbox', 'Position', [81,54+distV*7/2,130,23],...
                   'Value', 1, 'String', 'Paint Room', 'Callback', {@update3DPointS});
               
    bRoomH = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81+130,54+distV*5,50,23],...
                  'Value', roomH, 'String', roomH, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81+130,25+distV*5,50,23],...
                    'String','Room Height','BackgroundColor',bgcolor);
                
    bNearClipPlane = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81+130,54+distV*4,50,23],...
                  'Value', nearClipPlaneDist, 'String', nearClipPlaneDist, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81+130,25+distV*4,50,23],...
                    'String','Near Clip Plane','BackgroundColor',bgcolor);
           
    bGridStep = uicontrol('Parent', f, 'Style', 'edit', 'Position', [81+130,54+distV*3,50,23],...
                  'Value', gridStep, 'String', gridStep, 'Callback', {@update3DPointS});
    uicontrol('Parent',f,'Style','text','Position',[81+130,25+distV*3,50,23],...
                    'String','Grid Step','BackgroundColor',bgcolor);
                
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
        heightLimitIdent = str2double(get(bLimitIdent,'String'));
        frustumCheck = get(bFrustum,'Value');
        gridRoomCheck = get(bGridRoom,'Value');
        cutRoomCheck = get(bCutRoom,'Value');
        paintRoomCheck = get(bPaintRoom,'Value');
        interCheck = get(bInter,'Value');
        normalsCheck = get(bNormals,'Value');
        heightInterCheck = get(bInterHeight,'Value');
        heightLimitInterCheck = get(bHeightLimitInter,'Value');
        interLimitIdentCheck = get(bInterLimitIdent,'Value');
        numberOfObjects = get(bNumberOfObjects,'Value') - 1;
        roomH = str2double(get(bRoomH,'String'));
        nearClipPlaneDist = str2double(get(bNearClipPlane,'String'));
        gridStep = str2double(get(bGridStep,'String'));
        camW = str2double(get(bCamW,'String'));
        camD = str2double(get(bCamD,'String'));
        camH = str2double(get(bCamH,'String'));
        
        panStr = "Pan " + pan;
        tiltStr = "Tilt " + tilt;
        rollStr = "Roll (unused) " + roll;
        
        set(bPanText,'String',panStr);
        set(bTiltText,'String',tiltStr);
        set(bRollText,'String',rollStr);
        
        camPos = [camPosX camPosY camPosZ];
        
        aspect = W / H;
        
        if fovH <= 0
            fovH = fovV * aspect;
        end

        if fovV <= 0
            fovV = fovH / aspect;
        end

        fovHTan = tan(fovH / 2 / 180 * pi);
        fovVTan = tan(fovV / 2 / 180 * pi);
        
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
        [upRightFar,upLeftFar,~,~] = findFrustumBase(fcpCenter,fovHTan,fovVTan,R,farClipPlane);
        
        room = cell(wallsCount + 1,1);              % Создание помещения
        roomSize = zeros(wallsCount,1);
        roofPtsOrder = zeros(wallsCount,2);
        for j = 1:wallsCount
            if j ~= wallsCount
                wall = getWall(wallsPts(j,:),wallsPts(j + 1,:),roomH);
                wallW = norm(wallsPts(j,:) - wallsPts(j + 1,:));

                roofPtsOrder(j,1) = j;
                roofPtsOrder(j,2) = j + 1;
            else
                wall = getWall(wallsPts(j,:),wallsPts(1,:),roomH);
                wallW = norm(wallsPts(j,:) - wallsPts(1,:));

                roofPtsOrder(j,1) = j;
                roofPtsOrder(j,2) = 1;
            end
            room{j,1} = wall;
            roomSize(j,1) = wallW;
        end

        DT = delaunayTriangulation(wallsPts,roofPtsOrder);
        isInside = isInterior(DT);      % Поиск треугольников внутри ограниченных рёбер
        tri = DT(isInside, :);          % Получить индексы точек внутренних треугольников
        roof = triangulation(tri,[wallsPts ones(wallsCount,1) * roomH]);
        room{wallsCount + 1,1} = roof;
        
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
        
        % Определение плоскости ограничения по высоте
        V1 = [0 1 heightLimit];                 % Любая точка на плоскости пола
        X1 = [0 0 heightLimit + 1];             % Вектор нормали плоскости
        
        % Определение плоскости допустимого ограничения по высоте для идентификации
        V2 = [0 1 heightLimitIdent];            % Любая точка на плоскости пола
        X2 = [0 0 heightLimitIdent + 1];        % Вектор нормали плоскости
        
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

        parallelepipeds = cell(maxNumberOfObjects,1);

        floorPoly = polyshape(planeInterFloor(:,1:2));
        
        nearClipPlane = camPos + T * nearClipPlaneDist;

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
            if interLimitIdentCheck
                X3 = X2;
                V3 = V2;
            else
                X3 = X;
                V3 = V0;
            end
            if j <= numberOfObjects 
                if isBehind(parallelepipeds{j,1},T,nearClipPlane,'full')
                    continue
                elseif isBehind(parallelepipeds{j,1},T,nearClipPlane,'any')
                        TRInterCamera = planeObjectIntersection(parallelepipeds{j,1},T,nearClipPlane);
                    if (isBehind(TRInterCamera,X3,V3,'full'))
                        continue
                    elseif (isBehind(TRInterCamera,X3,V3,'any'))
                        TRInter = planeObjectIntersection(TRInterCamera,X3,V3);
                        TRPlane = planeProjection(TRInter,X3,V3,T,camPos,upRightFar,upLeftFar);
                    else
                        TRPlane = planeProjection(TRInterCamera,X3,V3,T,camPos,upRightFar,upLeftFar);
                    end
                    unPoly = unionPolygons(TRPlane);
                    floorPoly = subtract(floorPoly,unPoly);
                else
                    if (isBehind(parallelepipeds{j,1},X3,V3,'full'))
                        continue
                    elseif (isBehind(parallelepipeds{j,1},X3,V3,'any'))
                        TRInter = planeObjectIntersection(parallelepipeds{j,1},X3,V3);
                        TRPlane = planeProjection(TRInter,X3,V3,T,camPos,upRightFar,upLeftFar);
                    else
                        TRPlane = planeProjection(parallelepipeds{j,1},X3,V3,T,camPos,upRightFar,upLeftFar);
                    end
                    unPoly = unionPolygons(TRPlane);
                    floorPoly = subtract(floorPoly,unPoly);
                end
            end
        end
        
        for j = 1:wallsCount
            if j == wallsCount
                if cutRoomCheck
                    if isBehind(room{j + 1,1},T,nearClipPlane,'full')

                    elseif isBehind(room{j + 1,1},T,nearClipPlane,'any')
                        room{j + 1,1} = planeObjectIntersection(room{j + 1,1},T,nearClipPlane);
                    end
                end
            end
            
            if isBehind(room{j,1},T,nearClipPlane,'full')
                continue
            elseif isBehind(room{j,1},T,nearClipPlane,'any')
                if cutRoomCheck
                    room{j,1} = planeObjectIntersection(room{j,1},T,nearClipPlane);
                else
                    TRInterCamera = planeObjectIntersection(room{j,1},T,nearClipPlane);
                end
                if (isBehind(room{j,1},X,V0,'full'))
                    continue
                elseif (isBehind(room{j,1},X,V0,'any'))
                    if cutRoomCheck
                        TRInter = planeObjectIntersection(room{j,1},X,V0);
                    else
                        TRInter = planeObjectIntersection(TRInterCamera,X,V0);
                    end
                    TRPlane = planeProjection(TRInter,X,V0,T,camPos,upRightFar,upLeftFar);
                else
                    if cutRoomCheck
                        TRPlane = planeProjection(room{j,1},X,V0,T,camPos,upRightFar,upLeftFar);
                    else
                        TRPlane = planeProjection(TRInterCamera,X,V0,T,camPos,upRightFar,upLeftFar);
                    end
                end
                unPoly = unionPolygons(TRPlane);
                floorPoly = subtract(floorPoly,unPoly);
            else
                if (isBehind(room{j,1},X,V0,'full'))
                    continue
                elseif (isBehind(room{j,1},X,V0,'any'))
                    TRInter = planeObjectIntersection(room{j,1},X,V0);
                    TRPlane = planeProjection(TRInter,X,V0,T,camPos,upRightFar,upLeftFar);
                else
                    TRPlane = planeProjection(room{j,1},X,V0,T,camPos,upRightFar,upLeftFar);
                end
                unPoly = unionPolygons(TRPlane);
                floorPoly = subtract(floorPoly,unPoly);
            end
        end
        
        % Удаление зоны видимости камеры снаружи помещения (не костыль)
        floorRoom = polyshape(wallsPts);
        floorPoly = intersect(floorPoly,floorRoom);
        if heightLimitInterCheck
            floorPoly = intersect(floorPoly,conjunction);
        end

        interIdentPoly = intersect(interIdentPoly,floorPoly);
        interRecogPoly = intersect(interRecogPoly,floorPoly);
        interVisibPoly = intersect(interVisibPoly,floorPoly);
        interDetectPoly = intersect(interDetectPoly,floorPoly);
        interMonitorPoly = intersect(interMonitorPoly,floorPoly);
        
        % Определение доступного места для установки камеры и вычисление сетки
        % точек возможных положений камеры (без учёта окон и дверей)
        gridWalls = cell(wallsCount,1);
        wallAvailableH = roomH - (2 * camH + heightLimit);
        pointsNumberH = floor(wallAvailableH / gridStep);
        if pointsNumberH == 0 && gridStep > wallAvailableH
            pointsNumberH = 1;
        end

        pointsH = zeros(pointsNumberH,1);
        for j = 1:pointsNumberH         % Общие координаты точек сетки по Z
            if pointsNumberH == 1
                pointsH(j,1) = camH + heightLimit + wallAvailableH / 2;
            else
                pointsH(j,1) = camH + heightLimit + gridStep * j;
            end
        end

        % Расчёт сетки точек для стен
        for j = 1:wallsCount
            wallW = roomSize(j,1);                              % Ширина стены
            wallAvailableW = wallW - 2 * camW;                  % Доступная часть стены
            pointsNumberW = floor(wallAvailableW / gridStep);   % Количество точек сетки
            if pointsNumberW == 0 && gridStep > wallAvailableW
                pointsNumberW = 1;
            end
            
            startPoint = wallsPts(roofPtsOrder(j,1),:);         % Начальная координата стены
            endPoint = wallsPts(roofPtsOrder(j,2),:);           % Конечная координата стены
            direction = endPoint - startPoint;                  % Вектор стены
            directionScaled = direction/norm(direction);        % Вектор длиной 1
            camWScaled = startPoint + directionScaled * camW;   % Начало сетки по ширине с учётом габаритов камеры

            pointsW = zeros(pointsNumberW,2);                   % Точки сетки по ширине
            for m = 1:pointsNumberW
                if pointsNumberW == 1
                    pointsW(m,:) = camWScaled + directionScaled * (wallAvailableW / 2);
                else
                    pointsW(m,:) = camWScaled + directionScaled * gridStep * m;
                end
            end

            gridWall = cell(pointsNumberH,1);                   % Точки сетки по высоте
            for m = 1:pointsNumberH
                gridWall{m,1} = [pointsW ones(pointsNumberW,1) * pointsH(m,1)];
            end

            gridWalls{j,1} = cat(1,gridWall{:});
        end

        % Расчёт сетки точек для потолка
        [rectX,rectY] = minBoundRect(wallsPts(:,1),wallsPts(:,2));  % Описание помещения прямоугольником
        roofStartPointW = [rectX(1,1) rectY(1,1)];
        roofStartPointD = [rectX(2,1) rectY(2,1)];
        roofEndPointD = [rectX(3,1) rectY(3,1)];
        roofW = norm(roofStartPointD - roofStartPointW);
        roofD = norm(roofEndPointD - roofStartPointD);

        roofAvailableW = roofW - 2 * camW;
        roofAvailableD = roofD - 2 * camW;
        roofPointsNumberW = floor(roofAvailableW / gridStep);
        roofPointsNumberD = floor(roofAvailableD / gridStep);
        
        if roofPointsNumberW == 0 && gridStep > roofAvailableW
            roofPointsNumberW = 1;
        end
        if roofPointsNumberD == 0 && gridStep > roofAvailableD
            roofPointsNumberD = 1;
        end

        roofDirectionW = roofStartPointD - roofStartPointW;     % Вектор одной стороны описанного прямоугольника 
        roofDirectionD = roofEndPointD - roofStartPointD;       % Вектор другой стороны описанного прямоугольника 
        roofDirectionWScaled = roofDirectionW/norm(roofDirectionW);     % Вектора длиной 1
        roofDirectionDScaled = roofDirectionD/norm(roofDirectionD);
        roofCamWScaled = roofStartPointW + roofDirectionWScaled * camW;

        roofPointsW = zeros(roofPointsNumberW,2);
        gridRoof = zeros(roofPointsNumberW * roofPointsNumberD,2);

        for j = 1:roofPointsNumberW
            if roofPointsNumberW == 1
                roofPointsW(j,:) = roofCamWScaled + roofDirectionWScaled * (roofAvailableW / 2);
            else
                roofPointsW(j,:) = roofCamWScaled + roofDirectionWScaled * gridStep * j;
            end
        end

        for j = 1:roofPointsNumberW
            roofCamDScaled = roofPointsW(j,:) + roofDirectionDScaled * camW;
            for m = 1:roofPointsNumberD
                if roofPointsNumberD == 1
                    gridRoof((j - 1) * roofPointsNumberD + m,:) = roofCamDScaled + roofDirectionDScaled * (roofAvailableD / 2);
                else
                    gridRoof((j - 1) * roofPointsNumberD + m,:) = roofCamDScaled + roofDirectionDScaled * gridStep * m;
                end
            end
        end

        inRoof = inpolygon(gridRoof(:,1),gridRoof(:,2),...      % Удаление точек, которые не попадают на потолок
            [wallsPts(:,1); wallsPts(1,1)],[wallsPts(:,2); wallsPts(1,2)]);
        for j = roofPointsNumberW * roofPointsNumberD:-1:1
            if ~inRoof(j) || cameraWallIntersect(gridRoof(j,1:2),wallsPts,camW)
                gridRoof(j,:) = [];
            end
        end

        [mGridRoof,~] = size(gridRoof);
        gridRoof = [gridRoof ones(mGridRoof,1) * roomH];
        
        % Перемещение polyshape на нужную высоту (на будущее)
        %     M = [   1   0   0   0
        %             0   1   0   0
        %             0   0   1   10
        %             0   0   0   1   ];
        %     t=hgtransform('Matrix',M);     
        %     Hpgon = plot(interFarPoly,'Parent',t,'FaceColor','r');

        % Если рёбра frustum пересекают плоскость пола, то строим пересечение на
        % графике
        if interCheck == 1
            interIdent.Visible = 'on';
            interRecog.Visible = 'on';
            interVisib.Visible = 'on';
            interDetect.Visible = 'on';
            interMonitor.Visible = 'on';
            
            [mIdent, ~] = size(interIdentPoly.Vertices);
            if (mIdent < 3)
                interIdent.Shape.Vertices = [];
            else
                interIdent.Shape.Vertices = interIdentPoly.Vertices;
            end
            
            [mRecog, ~] = size(interRecogPoly.Vertices);
            if (mRecog < 3)
                interRecog.Shape.Vertices = [];
            else
                interRecog.Shape.Vertices = interRecogPoly.Vertices;
            end
            
            [mVisib, ~] = size(interVisibPoly.Vertices);
            if (mVisib < 3)
                interVisib.Shape.Vertices = [];
            else
                interVisib.Shape.Vertices = interVisibPoly.Vertices;
            end
            
            [mDetect, ~] = size(interDetectPoly.Vertices);
            if (mDetect < 3)
                interDetect.Shape.Vertices = [];
            else
                interDetect.Shape.Vertices = interDetectPoly.Vertices;
            end
            
            [mMonitor, ~] = size(interMonitorPoly.Vertices);
            if (mMonitor < 3)
                interMonitor.Shape.Vertices = [];
            else
                interMonitor.Shape.Vertices = interMonitorPoly.Vertices;
            end
        else
            interIdent.Visible = 'off';
            interRecog.Visible = 'off';
            interVisib.Visible = 'off';
            interDetect.Visible = 'off';
            interMonitor.Visible = 'off';
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
        for j = 1:wallsCount + 1
            roomPlot{j,1}.Vertices = room{j,1}.Points;
            roomPlot{j,1}.Faces = room{j,1}.ConnectivityList;
            
            P1 = incenter(room{j,1});
            F1 = faceNormal(room{j,1});
            set(quiverPlot{j,1},'XData',P1(:,1),'YData',P1(:,2),'ZData',P1(:,3),...
                'UData',F1(:,1),'VData',F1(:,2),'WData',F1(:,3));
            if normalsCheck
                set(quiverPlot{j,1},'Visible','on');
            else
                set(quiverPlot{j,1},'Visible','off');
            end
            
            if paintRoomCheck
                roomPlot{j,1}.FaceColor = paintRoomColor;
            else
                roomPlot{j,1}.FaceColor = 'none';
            end
        end
        
        % Построение сетки помещения
        for j = 1:wallsCount
            if gridRoomCheck
                set(gridWallsPlot{j,1}, 'visible', 'on');
                set(gridWallsPlot{j,1},'XData',gridWalls{j,1}(:,1),'YData',gridWalls{j,1}(:,2),'ZData',gridWalls{j,1}(:,3));
            else
                set(gridWallsPlot{j,1}, 'visible', 'off');
            end
        end

        if gridRoomCheck
            set(gridRoofPlot, 'visible', 'on');
            set(gridRoofPlot,'XData',gridRoof(:,1),'YData',gridRoof(:,2),'ZData',gridRoof(:,3));
        else
            set(gridRoofPlot, 'visible', 'off');
        end

        % Переопределение построенных frustum`ов
        if frustumCheck == 1
            set(frustumUpIdent, 'visible', 'on');
            set(frustumLeftIdent, 'visible', 'on');
            set(frustumDownIdent, 'visible', 'on');
            set(frustumRightIdent, 'visible', 'on');
            PIdent = [upRightIdent; upLeftIdent; downLeftIdent; downRightIdent; camPos];
            set(frustumUpIdent, 'XData', PIdent(indUpF, 1), 'YData', PIdent(indUpF, 2), 'ZData', PIdent(indUpF, 3));
            set(frustumLeftIdent, 'XData', PIdent(indLeftF, 1), 'YData', PIdent(indLeftF, 2), 'ZData', PIdent(indLeftF, 3));
            set(frustumDownIdent, 'XData', PIdent(indDownF, 1), 'YData', PIdent(indDownF, 2), 'ZData', PIdent(indDownF, 3));
            set(frustumRightIdent, 'XData', PIdent(indRightF, 1), 'YData', PIdent(indRightF, 2), 'ZData', PIdent(indRightF, 3));
            
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
            set(frustumUpIdent, 'visible', 'off');
            set(frustumLeftIdent, 'visible', 'off');
            set(frustumDownIdent, 'visible', 'off');
            set(frustumRightIdent, 'visible', 'off');
            
            set(frustumUpRecog, 'visible', 'off');
            set(frustumLeftRecog, 'visible', 'off');
            set(frustumDownRecog, 'visible', 'off');
            set(frustumRightRecog, 'visible', 'off');
            
            set(frustumUpVisib, 'visible', 'off');
            set(frustumLeftVisib, 'visible', 'off');
            set(frustumDownVisib, 'visible', 'off');
            set(frustumRightVisib, 'visible', 'off');
            
            set(frustumUpDetect, 'visible', 'off');
            set(frustumLeftDetect, 'visible', 'off');
            set(frustumDownDetect, 'visible', 'off');
            set(frustumRightDetect, 'visible', 'off');
            
            set(frustumUpMonitor, 'visible', 'off');
            set(frustumLeftMonitor, 'visible', 'off');
            set(frustumDownMonitor, 'visible', 'off');
            set(frustumRightMonitor, 'visible', 'off');
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

    function bestCameraLocation(~,~)
        simpleAlgorithm = get(bSimpleAlgorithm,'Value');
        
        if (fovH >= 180)
            possibleRotateH = 1;
        else
            possibleRotateH = 180 - fovH;
        end

        if (fovV >= 90)
            possibleRotateV = 1;
        else
            possibleRotateV = 90 - fovV;
        end 
        
        bestLocation = cell(5,1);
        bestWallsLocation = cell(wallsCount,1);
        
        fprintf("Диапазон по горизонтали - %d, диапазон по вертикали - %d\n",...
                possibleRotateH,possibleRotateV);

        tFrustum = 0;                   % Время нахождения координат точек основания frustum
        tPlaneInter = 0;                % Время нахождения точек пересечения frustum с плоскостью пола
        tFrustumCount = 0;              % Счётчик для frustum
        tInterCameraP = 0;              % Время разрезания объекта плоскостью камеры
        tInterCameraPCount = 0;         % Счётчик для tInterCameraP
        tInterCameraW = 0;              % Время разрезания стены плоскостью камеры
        tInterCameraWCount = 0;         % Счётчик для tInterCameraW
        tHeightLimitSimple = 0;         % Время разрезания объекта плоскостью ограничения идентификации (простой)
        tHeightLimitSimpleCount = 0;    % Счётчик для tHeightLimitSimple
        tHeightLimit = 0;               % Время разрезания объекта плоскостью ограничения идентификации (универсальный)
        tHeightLimitCount = 0;          % Счётчик для tHeightLimitSimple
        tPlaneProjectP = 0;             % Время проекции объекта на плоскость
        tPlaneProjectPCount = 0;        % Счётчик для tPlaneProjectP
        tUnionPolygons = 0;             % Время объединения полигонов на полу
        tUnionPolygonsCount = 0;        % Счётчик для tUnionPolygons
        tPlaneProjectW = 0;             % Время проекции стены на плоскость
        tPlaneProjectWCount = 0;        % Счётчик для tPlaneProjectW
        tInterPoly = 0;                 % Время вписывания всех зон видимости в комнату 
        tInterPolyCount = 0;            % Счётчик для tInterPoly
        tLocation = 0;                  % Время создания конструкции для хранения данных
        tLocationCount = 0;             % Счётчик для tLocation
        tStart = tic;                   % Начальное включения работы всего алгоритма
        
        parfor m = 1:wallsCount
            bestWallLocation = cell(5,1);
            
            srartWall = wallsPts(roofPtsOrder(m,1),:);
            endWall = wallsPts(roofPtsOrder(m,2),:);
            wallDirect = endWall - srartWall;
            angle = atan2d(wallDirect(1), wallDirect(2));   % Угол стены относительно Y
            
            F = faceNormal(room{m,1},1);
            
            [mGridPoints,~] = size(gridWalls{m,1});
            
            stepH = 5;
            stepV = 3;
            
            for g = 1:mGridPoints
                if possibleRotateH == 1
                    currentCamAngleH = angle - 90;
                else
                    currentCamAngleH = angle - possibleRotateH - fovH / 2;
                end
                
                for h = 1:stepH:possibleRotateH
                    if possibleRotateV == 1
                        currentCamAngleV = 45;
                    else
                        currentCamAngleV = fovV / 2;
                    end
                    
                    for v = 1:stepV:possibleRotateV
                        R1 = findRotationMatrix(currentCamAngleH,currentCamAngleV,0);
                        
                        camPos1 = gridWalls{m,1}(g,:) + F * camD;

                        T1 = X * R1;                                    % Вектор направления камеры

                        identCenter1 = camPos1 + T1 * identDist;        % Центр основания frustum'a идентификации
                        recogCenter1 = camPos1 + T1 * recogDist;        % Центр основания frustum'a распознования
                        visibCenter1 = camPos1 + T1 * visibDist;        % Центр основания frustum'a обзора
                        detectCenter1 = camPos1 + T1 * detectDist;      % Центр основания frustum'a детекции
                        monitorCenter1 = camPos1 + T1 * monitorDist;    % Центр основания frustum'a мониторинга
                        fcpCenter1 = camPos1 + T1 * farClipPlane;       % Центр основания дальнего frustum

                        tFrustumStart = tic;
                        % Координаты точек основания frustum
                        [upRightIdent1,upLeftIdent1,downRightIdent1,downLeftIdent1] = ...
                            findFrustumBase(identCenter1,fovHTan,fovVTan,R1,identDist);
                        [upRightRecog1,upLeftRecog1,downRightRecog1,downLeftRecog1] = ...
                            findFrustumBase(recogCenter1,fovHTan,fovVTan,R1,recogDist);
                        [upRightVisib1,upLeftVisib1,downRightVisib1,downLeftVisib1] = ...
                            findFrustumBase(visibCenter1,fovHTan,fovVTan,R1,visibDist);
                        [upRightDetect1,upLeftDetect1,downRightDetect1,downLeftDetect1] = ...
                            findFrustumBase(detectCenter1,fovHTan,fovVTan,R1,detectDist);
                        [upRightMonitor1,upLeftMonitor1,downRightMonitor1,downLeftMonitor1] = ...
                            findFrustumBase(monitorCenter1,fovHTan,fovVTan,R1,monitorDist);
                        [upRightFar1,upLeftFar1,~,~] = findFrustumBase(fcpCenter1,fovHTan,fovVTan,R1,farClipPlane);
                        tFrustum = tFrustum + toc(tFrustumStart);

                        tPlaneInterStart = tic;
                        % Нахождение точек пересечения frustum с плоскостью пола
                        planeInterIdent1 = planeFrustumIntersect(X,V0,camPos1,...
                            upRightIdent1,upLeftIdent1,downRightIdent1,downLeftIdent1);
                        planeInterRecog1 = planeTruncFrustumIntersect(X,V0,...
                            upRightIdent1,upLeftIdent1,downRightIdent1,downLeftIdent1,...
                            upRightRecog1,upLeftRecog1,downRightRecog1,downLeftRecog1);
                        planeInterVisib1 = planeTruncFrustumIntersect(X,V0,...
                            upRightRecog1,upLeftRecog1,downRightRecog1,downLeftRecog1,...
                            upRightVisib1,upLeftVisib1,downRightVisib1,downLeftVisib1);
                        planeInterDetect1 = planeTruncFrustumIntersect(X,V0,...
                            upRightVisib1,upLeftVisib1,downRightVisib1,downLeftVisib1,...
                            upRightDetect1,upLeftDetect1,downRightDetect1,downLeftDetect1);
                        planeInterMonitor1 = planeTruncFrustumIntersect(X,V0,...
                            upRightDetect1,upLeftDetect1,downRightDetect1,downLeftDetect1,...
                            upRightMonitor1,upLeftMonitor1,downRightMonitor1,downLeftMonitor1);
                        tPlaneInter = tPlaneInter + toc(tPlaneInterStart);
                        tFrustumCount = tFrustumCount + 1;
                        
                        % Нахождение точек пересечения полного frustum с плоскостью ограничения
                        % по высоте
                        planeInterLimit1 = planeFrustumIntersect(X1,V1,camPos1,...
                            upRightMonitor1,upLeftMonitor1,downRightMonitor1,downLeftMonitor1);

                        % Нахождение точек пересечения полного frustum с плоскостью пола
                        planeInterFloor1 = planeFrustumIntersect(X,V0,camPos1,...
                            upRightMonitor1,upLeftMonitor1,downRightMonitor1,downLeftMonitor1);

                        % Нахождения конъюнкции многоугольников на высоте ограничения и на полу
                        conjunction1 = planesConjunction(planeInterLimit1,planeInterFloor1);

                        % Создание объектов и нахождение слепых зон
                        interIdentPoly1 = polyshape(planeInterIdent1(:,1:2));
                        interRecogPoly1 = polyshape(planeInterRecog1(:,1:2));
                        interVisibPoly1 = polyshape(planeInterVisib1(:,1:2));
                        interDetectPoly1 = polyshape(planeInterDetect1(:,1:2));
                        interMonitorPoly1 = polyshape(planeInterMonitor1(:,1:2));

                        floorPoly1 = polyshape(planeInterFloor1(:,1:2));

                        nearClipPlane1 = camPos1 + T1 * nearClipPlaneDist;

                        for j = 1:maxNumberOfObjects
                            X3 = X2;
                            V3 = V2;
                            if j <= numberOfObjects 
                                if isBehind(parallelepipeds{j,1},T1,nearClipPlane1,'full')
                                    continue
                                elseif isBehind(parallelepipeds{j,1},T1,nearClipPlane1,'any')
                                    tInterCameraPStart = tic;
                                    TRInterCamera1 = planeObjectIntersection(parallelepipeds{j,1},T1,nearClipPlane1);
                                    tInterCameraP = tInterCameraP + toc(tInterCameraPStart);
                                    tInterCameraPCount = tInterCameraPCount + 1;
                                    if (isBehind(TRInterCamera1,X3,V3,'full'))
                                        continue
                                    elseif (isBehind(TRInterCamera1,X3,V3,'any'))
                                        if simpleAlgorithm
                                            tHeightLimitSimpleStart = tic;
                                            TRInter1 = liftParallelepipedBase(TRInterCamera1,V3);
                                            tHeightLimitSimple = tHeightLimitSimple + toc(tHeightLimitSimpleStart);
                                            tHeightLimitSimpleCount = tHeightLimitSimpleCount + 1;
                                        else
                                            tHeightLimitStart = tic;
                                            TRInter1 = planeObjectIntersection(TRInterCamera1,X3,V3);
                                            tHeightLimit = tHeightLimit + toc(tHeightLimitStart);
                                            tHeightLimitCount = tHeightLimitCount + 1;
                                        end
                                        tPlaneProjectPStart = tic;
                                        TRPlane1 = planeProjection(TRInter1,X3,V3,T1,camPos1,upRightFar1,upLeftFar1);
                                        tPlaneProjectP = tPlaneProjectP + toc(tPlaneProjectPStart);
                                        tPlaneProjectPCount = tPlaneProjectPCount + 1;
                                    else
                                        tPlaneProjectPStart = tic;
                                        TRPlane1 = planeProjection(TRInterCamera1,X3,V3,T1,camPos1,upRightFar1,upLeftFar1);
                                        tPlaneProjectP = tPlaneProjectP + toc(tPlaneProjectPStart);
                                        tPlaneProjectPCount = tPlaneProjectPCount + 1;
                                    end
                                    tUnionPolygonsStart = tic;
                                    unPoly1 = unionPolygons(TRPlane1);
                                    tUnionPolygons = tUnionPolygons + toc(tUnionPolygonsStart);
                                    tUnionPolygonsCount = tUnionPolygonsCount + 1;
                                    floorPoly1 = subtract(floorPoly1,unPoly1);
                                else
                                    if (isBehind(parallelepipeds{j,1},X3,V3,'full'))
                                        continue
                                    elseif (isBehind(parallelepipeds{j,1},X3,V3,'any'))
                                        if simpleAlgorithm
                                            tHeightLimitSimpleStart = tic;
                                            TRInter1 = liftParallelepipedBase(parallelepipeds{j,1},V3);
                                            tHeightLimitSimple = tHeightLimitSimple + toc(tHeightLimitSimpleStart);
                                            tHeightLimitSimpleCount = tHeightLimitSimpleCount + 1;
                                        else
                                            tHeightLimitStart = tic;
                                            TRInter1 = planeObjectIntersection(parallelepipeds{j,1},X3,V3);
                                            tHeightLimit = tHeightLimit + toc(tHeightLimitStart);
                                            tHeightLimitCount = tHeightLimitCount + 1;
                                        end
                                        tPlaneProjectPStart = tic;
                                        TRPlane1 = planeProjection(TRInter1,X3,V3,T1,camPos1,upRightFar1,upLeftFar1);
                                        tPlaneProjectP = tPlaneProjectP + toc(tPlaneProjectPStart);
                                        tPlaneProjectPCount = tPlaneProjectPCount + 1;
                                    else
                                        tPlaneProjectPStart = tic;
                                        TRPlane1 = planeProjection(parallelepipeds{j,1},X3,V3,T1,camPos1,upRightFar1,upLeftFar1);
                                        tPlaneProjectP = tPlaneProjectP + toc(tPlaneProjectPStart);
                                        tPlaneProjectPCount = tPlaneProjectPCount + 1;
                                    end
                                    tUnionPolygonsStart = tic;
                                    unPoly1 = unionPolygons(TRPlane1);
                                    tUnionPolygons = tUnionPolygons + toc(tUnionPolygonsStart);
                                    tUnionPolygonsCount = tUnionPolygonsCount + 1;
                                    floorPoly1 = subtract(floorPoly1,unPoly1);
                                end
                            end
                        end

                        for j = 1:wallsCount
                            if isBehind(room{j,1},T1,nearClipPlane1,'full')
                                continue
                            elseif isBehind(room{j,1},T1,nearClipPlane1,'any')
                                tInterCameraWStart = tic;
                                TRInterCamera1 = planeObjectIntersection(room{j,1},T1,nearClipPlane1);
                                tInterCameraW = tInterCameraW + toc(tInterCameraWStart);
                                tInterCameraWCount = tInterCameraWCount + 1;
                                if (isBehind(room{j,1},X,V0,'full'))
                                    continue
                                elseif (isBehind(room{j,1},X,V0,'any'))
                                    TRInter1 = planeObjectIntersection(TRInterCamera1,X,V0);
                                    TRPlane1 = planeProjection(TRInter1,X,V0,T1,camPos1,upRightFar1,upLeftFar1);
                                else
                                    tPlaneProjectWStart = tic;
                                    TRPlane1 = planeProjection(TRInterCamera1,X,V0,T1,camPos1,upRightFar1,upLeftFar1);
                                    tPlaneProjectW = tPlaneProjectW + toc(tPlaneProjectWStart);
                                    tPlaneProjectWCount = tPlaneProjectWCount + 1;
                                end
                                tUnionPolygonsStart = tic;
                                unPoly1 = unionPolygons(TRPlane1);
                                tUnionPolygons = tUnionPolygons + toc(tUnionPolygonsStart);
                                tUnionPolygonsCount = tUnionPolygonsCount + 1;
                                floorPoly1 = subtract(floorPoly1,unPoly1);
                            else
                                if (isBehind(room{j,1},X,V0,'full'))
                                    continue
                                elseif (isBehind(room{j,1},X,V0,'any'))
                                    TRInter1 = planeObjectIntersection(room{j,1},X,V0);
                                    TRPlane1 = planeProjection(TRInter1,X,V0,T1,camPos1,upRightFar1,upLeftFar1);
                                else
                                    tPlaneProjectWStart = tic;
                                    TRPlane1 = planeProjection(room{j,1},X,V0,T1,camPos1,upRightFar1,upLeftFar1);
                                    tPlaneProjectW = tPlaneProjectW + toc(tPlaneProjectWStart);
                                    tPlaneProjectWCount = tPlaneProjectWCount + 1;
                                end
                                tUnionPolygonsStart = tic;
                                unPoly1 = unionPolygons(TRPlane1);
                                tUnionPolygons = tUnionPolygons + toc(tUnionPolygonsStart);
                                tUnionPolygonsCount = tUnionPolygonsCount + 1;
                                floorPoly1 = subtract(floorPoly1,unPoly1);
                            end
                        end

                        % Удаление зоны видимости камеры снаружи помещения (не костыль)
                        floorRoom1 = polyshape(wallsPts);
                        floorPoly1 = intersect(floorPoly1,floorRoom1);
                        
                        floorPoly1 = intersect(floorPoly1,conjunction1);
                        
                        tInterPolyStart = tic;
                        interIdentPoly1 = intersect(interIdentPoly1,floorPoly1);
                        interRecogPoly1 = intersect(interRecogPoly1,floorPoly1);
                        interVisibPoly1 = intersect(interVisibPoly1,floorPoly1);
                        interDetectPoly1 = intersect(interDetectPoly1,floorPoly1);
                        interMonitorPoly1 = intersect(interMonitorPoly1,floorPoly1);
                        tInterPoly = tInterPoly + toc(tInterPolyStart);
                        tInterPolyCount = tInterPolyCount + 1;
                        
                        tLocationStart = tic;
                        % Сохраняем все нужные данные для визуализации
                        Location = struct;
                        Location.area = area(floorPoly1);
                        Location.camPos = camPos1;
                        Location.frustumIdent = [upRightIdent1; upLeftIdent1; downLeftIdent1; downRightIdent1; camPos1];
                        Location.frustumRecog = [upRightIdent1; upLeftIdent1; downLeftIdent1; downRightIdent1; ...
                            upRightRecog1; upLeftRecog1; downLeftRecog1; downRightRecog1];
                        Location.frustumVisib = [upRightRecog1; upLeftRecog1; downLeftRecog1; downRightRecog1; ...
                            upRightVisib1; upLeftVisib1; downLeftVisib1; downRightVisib1];
                        Location.frustumDetect = [upRightVisib1; upLeftVisib1; downLeftVisib1; downRightVisib1; ...
                            upRightDetect1; upLeftDetect1; downLeftDetect1; downRightDetect1];
                        Location.frustumMonitor = [upRightDetect1; upLeftDetect1; downLeftDetect1; downRightDetect1; ...
                            upRightMonitor1; upLeftMonitor1; downLeftMonitor1; downRightMonitor1];
                        if currentCamAngleH > 0
                            Location.pan = currentCamAngleH;
                        else
                            Location.pan = 360 + currentCamAngleH;
                        end
                        Location.tilt = currentCamAngleV;
                        Location.roll = 0;
                        Location.interIdent = interIdentPoly1;
                        Location.interRecog = interRecogPoly1;
                        Location.interVisib = interVisibPoly1;
                        Location.interDetect = interDetectPoly1;
                        Location.interMonitor = interMonitorPoly1;
                        tLocation = tLocation + toc(tLocationStart);
                        tLocationCount = tLocationCount + 1;
                        
                        empty = false;
                        compare = false;
                        for b = 1:5   
                            if isempty(bestWallLocation{b,1})
                                bestWallLocation{b,1} = Location;
                                empty = true;
                                break;
                            elseif Location.area >= bestWallLocation{b,1}.area
                                compare = true;
                                break;
                            end
                        end
                        
                        if ~empty && compare
                            switch b
                                case 1
                                    bestWallLocation{5,1} = bestWallLocation{4,1};
                                    bestWallLocation{4,1} = bestWallLocation{3,1};
                                    bestWallLocation{3,1} = bestWallLocation{2,1};
                                    bestWallLocation{2,1} = bestWallLocation{1,1};
                                    bestWallLocation{1,1} = Location;
                                case 2
                                    bestWallLocation{5,1} = bestWallLocation{4,1};
                                    bestWallLocation{4,1} = bestWallLocation{3,1};
                                    bestWallLocation{3,1} = bestWallLocation{2,1};
                                    bestWallLocation{2,1} = Location;
                                case 3
                                    bestWallLocation{5,1} = bestWallLocation{4,1};
                                    bestWallLocation{4,1} = bestWallLocation{3,1};
                                    bestWallLocation{3,1} = Location;
                                case 4
                                    bestWallLocation{5,1} = bestWallLocation{4,1};
                                    bestWallLocation{4,1} = Location;
                                case 5
                                    bestWallLocation{5,1} = Location;
                            end
                        end
                            
                        currentCamAngleV = currentCamAngleV + stepV;
                    end
                    
                    currentCamAngleH = currentCamAngleH + stepH;
                end
            end
            
            bestWallsLocation{m,1} = bestWallLocation;
        end
        
        for m = 1:wallsCount
            empty = false;
            compare = false;
            for a = 1:5
                for b = 1:5   
                    if isempty(bestLocation{b,1})
                        bestLocation{b,1} = bestWallsLocation{m,1}{a,1};
                        empty = true;
                        break;
                    elseif bestWallsLocation{m,1}{a,1}.area >= bestLocation{b,1}.area
                        compare = true;
                        break;
                    end
                end

                if ~empty && compare
                    switch b
                        case 1
                            bestLocation{5,1} = bestLocation{4,1};
                            bestLocation{4,1} = bestLocation{3,1};
                            bestLocation{3,1} = bestLocation{2,1};
                            bestLocation{2,1} = bestLocation{1,1};
                            bestLocation{1,1} = bestWallsLocation{m,1}{a,1};
                        case 2
                            bestLocation{5,1} = bestLocation{4,1};
                            bestLocation{4,1} = bestLocation{3,1};
                            bestLocation{3,1} = bestLocation{2,1};
                            bestLocation{2,1} = bestWallsLocation{m,1}{a,1};
                        case 3
                            bestLocation{5,1} = bestLocation{4,1};
                            bestLocation{4,1} = bestLocation{3,1};
                            bestLocation{3,1} = bestWallsLocation{m,1}{a,1};
                        case 4
                            bestLocation{5,1} = bestLocation{4,1};
                            bestLocation{4,1} = bestWallsLocation{m,1}{a,1};
                        case 5
                            bestLocation{5,1} = bestWallsLocation{m,1}{a,1};
                    end
                end
            end
        end
        
        camPos = bestLocation{1,1}.camPos;
        PIdent = bestLocation{1,1}.frustumIdent;
        PRecog = bestLocation{1,1}.frustumRecog;
        PVisib = bestLocation{1,1}.frustumVisib;
        PDetect = bestLocation{1,1}.frustumDetect;
        PMonitor = bestLocation{1,1}.frustumMonitor;
        pan = bestLocation{1,1}.pan;
        tilt = bestLocation{1,1}.tilt;
        roll = bestLocation{1,1}.roll;
        interIdentPoly = bestLocation{1,1}.interIdent;
        interRecogPoly = bestLocation{1,1}.interRecog;
        interVisibPoly = bestLocation{1,1}.interVisib;
        interDetectPoly = bestLocation{1,1}.interDetect;
        interMonitorPoly = bestLocation{1,1}.interMonitor;
        
        set(bCamPosX,'String',camPos(1,1));
        set(bCamPosY,'String',camPos(1,2));
        set(bCamPosZ,'String',camPos(1,3));
        
        R = findRotationMatrix(pan,tilt,roll);
        pose = rigid3d(R,camPos);
        cam.AbsolutePose = pose;

        set(bPan,'Value',pan);
        set(bTilt,'Value',tilt);
        set(bRoll,'Value',roll);
        panStr = "Pan " + pan;
        tiltStr = "Tilt " + tilt;
        rollStr = "Roll (unused) " + roll;
        set(bPanText,'String',panStr);
        set(bTiltText,'String',tiltStr);
        set(bRollText,'String',rollStr);

        set(frustumUpIdent, 'XData', PIdent(indUpF, 1), 'YData', PIdent(indUpF, 2), 'ZData', PIdent(indUpF, 3));
        set(frustumLeftIdent, 'XData', PIdent(indLeftF, 1), 'YData', PIdent(indLeftF, 2), 'ZData', PIdent(indLeftF, 3));
        set(frustumDownIdent, 'XData', PIdent(indDownF, 1), 'YData', PIdent(indDownF, 2), 'ZData', PIdent(indDownF, 3));
        set(frustumRightIdent, 'XData', PIdent(indRightF, 1), 'YData', PIdent(indRightF, 2), 'ZData', PIdent(indRightF, 3));

        set(frustumUpRecog, 'XData', PRecog(indUp, 1), 'YData', PRecog(indUp, 2), 'ZData', PRecog(indUp, 3));
        set(frustumLeftRecog, 'XData', PRecog(indLeft, 1), 'YData', PRecog(indLeft, 2), 'ZData', PRecog(indLeft, 3));
        set(frustumDownRecog, 'XData', PRecog(indDown, 1), 'YData', PRecog(indDown, 2), 'ZData', PRecog(indDown, 3));
        set(frustumRightRecog, 'XData', PRecog(indRight, 1), 'YData', PRecog(indRight, 2), 'ZData', PRecog(indRight, 3));

        set(frustumUpVisib, 'XData', PVisib(indUp, 1), 'YData', PVisib(indUp, 2), 'ZData', PVisib(indUp, 3));
        set(frustumLeftVisib, 'XData', PVisib(indLeft, 1), 'YData', PVisib(indLeft, 2), 'ZData', PVisib(indLeft, 3));
        set(frustumDownVisib, 'XData', PVisib(indDown, 1), 'YData', PVisib(indDown, 2), 'ZData', PVisib(indDown, 3));
        set(frustumRightVisib, 'XData', PVisib(indRight, 1), 'YData', PVisib(indRight, 2), 'ZData', PVisib(indRight, 3));

        set(frustumUpDetect, 'XData', PDetect(indUp, 1), 'YData', PDetect(indUp, 2), 'ZData', PDetect(indUp, 3));
        set(frustumLeftDetect, 'XData', PDetect(indLeft, 1), 'YData', PDetect(indLeft, 2), 'ZData', PDetect(indLeft, 3));
        set(frustumDownDetect, 'XData', PDetect(indDown, 1), 'YData', PDetect(indDown, 2), 'ZData', PDetect(indDown, 3));
        set(frustumRightDetect, 'XData', PDetect(indRight, 1), 'YData', PDetect(indRight, 2), 'ZData', PDetect(indRight, 3));

        set(frustumUpMonitor, 'XData', PMonitor(indUp, 1), 'YData', PMonitor(indUp, 2), 'ZData', PMonitor(indUp, 3));
        set(frustumLeftMonitor, 'XData', PMonitor(indLeft, 1), 'YData', PMonitor(indLeft, 2), 'ZData', PMonitor(indLeft, 3));
        set(frustumDownMonitor, 'XData', PMonitor(indDown, 1), 'YData', PMonitor(indDown, 2), 'ZData', PMonitor(indDown, 3));
        set(frustumRightMonitor, 'XData', PMonitor(indRight, 1), 'YData', PMonitor(indRight, 2), 'ZData', PMonitor(indRight, 3));
        
        [mIdent, ~] = size(interIdentPoly.Vertices);
        if (mIdent < 3)
            interIdent.Shape.Vertices = [];
        else
            interIdent.Shape.Vertices = interIdentPoly.Vertices;
        end

        [mRecog, ~] = size(interRecogPoly.Vertices);
        if (mRecog < 3)
            interRecog.Shape.Vertices = [];
        else
            interRecog.Shape.Vertices = interRecogPoly.Vertices;
        end

        [mVisib, ~] = size(interVisibPoly.Vertices);
        if (mVisib < 3)
            interVisib.Shape.Vertices = [];
        else
            interVisib.Shape.Vertices = interVisibPoly.Vertices;
        end

        [mDetect, ~] = size(interDetectPoly.Vertices);
        if (mDetect < 3)
            interDetect.Shape.Vertices = [];
        else
            interDetect.Shape.Vertices = interDetectPoly.Vertices;
        end

        [mMonitor, ~] = size(interMonitorPoly.Vertices);
        if (mMonitor < 3)
            interMonitor.Shape.Vertices = [];
        else
            interMonitor.Shape.Vertices = interMonitorPoly.Vertices;
        end
        
        for j = 1:5
            disp('======================================================');
            disp(['Топ ', num2str(j), ':']);
            disp('======================================================');
            disp(['Площадь покрытия: ',num2str(bestLocation{j,1}.area)]);
            disp("Позиция камеры:");
            disp(bestLocation{j,1}.camPos);
            disp(['Pan: ',num2str(bestLocation{j,1}.pan)]);
            disp(['Tilt: ',num2str(bestLocation{j,1}.tilt)]);
            disp(['Roll: ',num2str(bestLocation{j,1}.roll)]);
        end
        
        tEnd = toc(tStart);
        
        tFrustum = tFrustum / tFrustumCount;
        tPlaneInter = tPlaneInter / tFrustumCount;
        tInterCameraP = tInterCameraP / tInterCameraPCount;
        tInterCameraW = tInterCameraW / tInterCameraWCount;
        tHeightLimitSimple = tHeightLimitSimple / tHeightLimitSimpleCount;
        tHeightLimit = tHeightLimit / tHeightLimitCount;
        tPlaneProjectP = tPlaneProjectP / tPlaneProjectPCount;
        tUnionPolygons = tUnionPolygons / tUnionPolygonsCount;
        tPlaneProjectW = tPlaneProjectW / tPlaneProjectWCount;
        tInterPoly = tInterPoly / tInterPolyCount;
        tLocation = tLocation / tLocationCount;
        
        disp('======================================================');
        disp('Время выполнения (в секундах)');
        disp('======================================================');
        disp(['Нахождение координат точек основания frustum: ',...
            num2str(tFrustum)]);
        disp(['Нахождение точек пересечения frustum с плоскостью пола: ',...
            num2str(tPlaneInter)]);
        disp(['Разрезание объекта (parallelepiped) плоскостью камеры (nearClipPlane): ',...
            num2str(tInterCameraP)]);
        disp(['Разрезание стены (wall) плоскостью камеры (nearClipPlane): ',...
            num2str(tInterCameraW)]);
        if simpleAlgorithm
            disp(['Разрезание объекта плоскостью ограничения идентификации (простой): ',...
                num2str(tHeightLimitSimple)]);
        else
            disp(['Разрезание объекта плоскостью ограничения идентификации (универсальный): ',...
                num2str(tHeightLimit)]);
        end
        disp(['Проекция объекта (parallelepiped) на плоскость: ',...
            num2str(tPlaneProjectP)]);
        disp(['Проекция стены (wall) на плоскость: ',...
            num2str(tPlaneProjectW)]);
        disp(['Объединение полигонов на полу: ',...
            num2str(tUnionPolygons)]);
        disp(['Вписывание всех зон видимости в комнату: ',...
            num2str(tInterPoly)]);
        disp(['Создание конструкции для хранения данных: ',...
            num2str(tLocation)]);
        disp(['Общее время выполнения: ',...
            num2str(tEnd)]);
    end
end