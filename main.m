% Пример камеры видеонаблюдения взят отсюда (2 сверху): 
% http://mikstmarine.ru/marine-cameras-specifications.html
% Габариты видокамеры взяты отсюда:
% https://www.citilink.ru/product/videokamera-ip-hikvision-ds-2cd2623g0-izs-1080p-2-8-12-mm-belyi-1081024/properties/
W = 3840;                   % Ширина изображения с камеры в пикселах
H = 2160;                   % Высота изображения с камеры в пикселах
pan = 86;                   % Угол поворота вокруг оси Z
tilt = 56;                  % Угол поворота вокруг оси X
roll = 0;                   % Угол поворота вокруг оси Y
fovH = 93;                  % Угол обзора по горизонтали
fovV = 51;                  % Угол обзора по вертикали
farClipPlane = 10000;       % Расстояние от камеры до дальней плоскости отсечения по оси Z
[roomH,gridStep,wallsPts,doorsSpec,windowsSpec,camPos] = loadRoom('room.txt'); % Параметры помещения и положения камеры
heightLimit = 2;            % Ограничение по высоте
heightLimitIdent = 0.5;     % Допустимое ограничение по высоте для идентификации
numberOfObjects = 0;        % Количество объектов (максимум 5)
camW = 0.145;               % Ширина камеры
camD = 0.333;               % Глубина камеры
camH = 0.145;               % Высота камеры
nearClipPlaneDist = 1e-1;   % Расстояние до ближней плоскости сечения камеры
camDist = 15;               % Допустимое расстояние до камеры от источника света
angleOfLight = 30;          % Допустимый угол между источником света и камерой
doorsPriority = 1.2;        % Приоритет для выбора положения камеры относительно дверей
stepH = 1;                  % Шаг в градусах по горизонтали при работе алгоритма
stepV = 1;                  % Шаг в градусах по вертикали при работе алгоритма

plotFrustumIntersect(W,H,pan,tilt,roll,fovH,fovV,...
    farClipPlane,camPos,heightLimit,heightLimitIdent,numberOfObjects,...
    wallsPts,roomH,doorsSpec,windowsSpec,gridStep,camW,camD,camH,nearClipPlaneDist,...
    camDist,angleOfLight,doorsPriority,stepH,stepV)