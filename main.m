% Пример камеры видеонаблюдения взят отсюда (2 сверху): 
% http://mikstmarine.ru/marine-cameras-specifications.html
% Габариты видокамеры взяты отсюда:
% https://www.citilink.ru/product/videokamera-ip-hikvision-ds-2cd2623g0-izs-1080p-2-8-12-mm-belyi-1081024/properties/
W = 1920;               % Ширина изображения с камеры в пикселах
H = 1080;               % Высота изображения с камеры в пикселах
pan = 0;                % Угол поворота вокруг оси Z
tilt = 50;              % Угол поворота вокруг оси X
roll = 0;               % Угол поворота вокруг оси Y
fovH = 93;              % Угол обзора по горизонтали
fovV = 51;              % Угол обзора по вертикали
farClipPlane = 30;      % Расстояние от камеры до дальней плоскости отсечения по оси Z
camPos = [0 -30 15];    % Позиция камеры относительно мировых координат
% length = 150;           % Длина стороны плоскости пола
heightLimit = 2;        % Ограничение по высоте
numberOfObjects = 0;    % Количество объектов (максимум 5)
roomW = 30;             % Ширина помещения
roomD = 20;             % Глубина помещения
roomH = 4;              % Высота помещения
camW = 0.145;           % Ширина камеры
camD = 0.333;           % Глубина камеры
camH = 0.145;           % Высота камеры

plotFrustumIntersect(W,H,pan,tilt,roll,fovH,fovV,...
    farClipPlane,camPos,heightLimit,numberOfObjects,...
    roomW,roomD,roomH,camW,camD,camH)