% Пример камеры видеонаблюдения взят отсюда (2 сверху): 
% http://mikstmarine.ru/marine-cameras-specifications.html
W = 1920;               % Ширина изображения с камеры в пикселах
H = 1080;               % Высота изображения с камеры в пикселах
pan = 0;                % Угол поворота вокруг оси Z
tilt = 50;              % Угол поворота вокруг оси X
roll = 0;               % Угол поворота вокруг оси Y
fovH = 93;              % Угол обзора по горизонтали
fovV = 51;              % Угол обзора по вертикали
farClipPlane = 30;      % Расстояние от камеры до дальней плоскости отсечения по оси Z
camPos = [0 -30 15];    % Позиция камеры относительно мировых координат
length = 150;           % Длина стороны плоскости пола
heightLimit = 2;        % Ограничение по высоте
numberOfObjects = 2;    % Количество объектов (максимум 5)

plotFrustumIntersect(W,H,pan,tilt,roll,fovH,fovV,...
    farClipPlane,camPos,length,heightLimit,numberOfObjects)