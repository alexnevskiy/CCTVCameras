function planeFrustumIntersect(W,H,pan,tilt,roll,fovV,nearClipPlane,camPos,length)
    aspect = W / H;         % Соотношение сторон камеры
    
    fovVTan = tan(fovV / 2 / 180 * pi);
    % halfHeight = nearClipPlane * fovVTan;
    % halfWidth =  halfHeight / aspect;

    R = findRotationMatrix(pan,tilt,roll);      % Матрица поворота
    X = [0 0 1];                                % Вектор нормали (также просто ненулевой вектор)
    T = X * R;                                  % Вектор направления камеры

    ncpCenter = camPos + T * nearClipPlane;     % Центр основания frustum

    % Координаты точек основания frustum
    upRight = ncpCenter + shiftDirection(fovVTan, fovVTan / aspect, R) * nearClipPlane;
    upLeft = ncpCenter + shiftDirection(-fovVTan, fovVTan / aspect, R) * nearClipPlane;
    downRight = ncpCenter + shiftDirection(fovVTan, -fovVTan / aspect, R) * nearClipPlane;
    downLeft = ncpCenter + shiftDirection(-fovVTan, -fovVTan / aspect, R) * nearClipPlane;

    A = [ -length -length 0];   % Координаты точек пола
    B = [ -length length 0];
    C = [ length length 0];
    D = [ length -length 0];
    V0 = [0 1 0];               % Любая точка на плоскости пола

    % Нахождение точек пересечения frustum с плоскостью пола
    [posUpRight, checkUpRight] = planeLineIntersect(X, V0, camPos, upRight);
    [posUpLeft, checkUpLeft] = planeLineIntersect(X, V0, camPos, upLeft);
    [posDownRight, checkDownRight] = planeLineIntersect(X, V0, camPos, downRight);
    [posDownLeft, checkDownLeft] = planeLineIntersect(X, V0, camPos, downLeft);

    f = figure;

    % Если рёбра frustum пересекают плоскость пола, то строим пересечение на
    % графике
    if (checkUpRight == 1 || 3) && (checkUpLeft == 1 || 3) ...
            && (checkDownRight == 1 || 3) && (checkDownLeft == 1 || 3)
        intersectPos = [posUpRight; posUpLeft; posDownLeft; posDownRight];
        inter = fill3(intersectPos(:,1),intersectPos(:,2),intersectPos(:,3),'green');
        hold on
    end

    % Построение frustum
    P = [upRight; upLeft; downLeft; downRight; camPos];
    indUp = [1 2 5]; frustumUp = fill3(P(indUp, 1), P(indUp, 2), P(indUp, 3), 'red');
    hold on
    indLeft = [2 3 5]; frustumLeft = fill3(P(indLeft, 1), P(indLeft, 2), P(indLeft, 3), 'red');
    indDown = [3 4 5]; frustumDown = fill3(P(indDown, 1), P(indDown, 2), P(indDown, 3), 'red');
    indRight = [4 1 5]; frustumRight = fill3(P(indRight, 1), P(indRight, 2), P(indRight, 3), 'red');

    % Построение плоскости пола
    allPlainPts = [A;B;C;D];
    fill3(allPlainPts(:,1),allPlainPts(:,2),allPlainPts(:,3),'yellow');
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
                    'String','Clipping Plane','BackgroundColor',bgcolor);
    
    function update3DPointS(~,~)
        pan = get(bPan,'Value');
        tilt = get(bTilt,'Value');
        roll = get(bRoll,'Value');
        camPosX = str2double(get(bCamPosX,'String'));
        camPosY = str2double(get(bCamPosY,'String'));
        camPosZ = str2double(get(bCamPosZ,'String'));
        nearClipPlane = str2double(get(bNearClipPlane,'String'));
        
        camPos = [camPosX camPosY camPosZ];

        R = findRotationMatrix(pan,tilt,roll);
        pose = rigid3d(R,camPos);
        cam.AbsolutePose = pose; 

        X = [0 0 1];                                % Вектор нормали (также просто ненулевой вектор)
        T = X * R;                                  % Вектор направления камеры

        ncpCenter = camPos + T * nearClipPlane;     % Центр основания frustum

        % Координаты точек основания frustum
        upRight = ncpCenter + shiftDirection(fovVTan, fovVTan / aspect, R) * nearClipPlane;
        upLeft = ncpCenter + shiftDirection(-fovVTan, fovVTan / aspect, R) * nearClipPlane;
        downRight = ncpCenter + shiftDirection(fovVTan, -fovVTan / aspect, R) * nearClipPlane;
        downLeft = ncpCenter + shiftDirection(-fovVTan, -fovVTan / aspect, R) * nearClipPlane;

        [posUpRight, checkUpRight] = planeLineIntersect(X, V0, camPos, upRight);
        [posUpLeft, checkUpLeft] = planeLineIntersect(X, V0, camPos, upLeft);
        [posDownRight, checkDownRight] = planeLineIntersect(X, V0, camPos, downRight);
        [posDownLeft, checkDownLeft] = planeLineIntersect(X, V0, camPos, downLeft);

        intersectPos = [posUpRight; posUpLeft; posDownLeft; posDownRight];

        set(inter, 'XData', intersectPos(:,1), 'YData', intersectPos(:,2), 'ZData', intersectPos(:,3));

        P = [upRight; upLeft; downLeft; downRight; camPos];
        set(frustumUp, 'XData', P(indUp, 1), 'YData', P(indUp, 2), 'ZData', P(indUp, 3));
        set(frustumLeft, 'XData', P(indLeft, 1), 'YData', P(indLeft, 2), 'ZData', P(indLeft, 3));
        set(frustumDown, 'XData', P(indDown, 1), 'YData', P(indDown, 2), 'ZData', P(indDown, 3));
        set(frustumRight, 'XData', P(indRight, 1), 'YData', P(indRight, 2), 'ZData', P(indRight, 3));
    end
end