% Пример камеры видеонаблюдения взят отсюда (2 сверху): 
% http://mikstmarine.ru/marine-cameras-specifications.html

% W = 1920;               % Ширина изображения
% H = 1080;               % Высота изображения
% flen = 3.6 * 10^(-3);   % Фокусное расстояние (раздел "Объектив")
pan = 0;                % Угол поворота вокруг оси Z
tilt = 45;              % Угол поворота вокруг оси X
roll = 0;               % Угол поворота вокруг оси Y
% fovH = 93;              % Угол обзора по горизонтали
fovV = 51;              % Угол обзора по вертикали
aspect = W / H;         % Соотношение сторон камеры
nearClipPlane = 80;     % Расстояние от камеры до ближайшей плоскости отсечения по оси Z
camPos = [0 -30 30];     % Позиция камеры относительно мировых координат

fovVTan = tan(fovV / 2 / 180 * pi);
halfHeight = nearClipPlane * fovVTan;
halfWidth =  halfHeight / aspect;

R = findRotationMatrix(pan,tilt,roll);      % Матрица поворота
X = [0 0 1];                                % Вектор нормали (также просто ненулевой вектор)
T = X * R;                                  % Вектор направления камеры

ncpCenter = camPos + T * nearClipPlane;     % Центр основания frustum

% Координаты точек основания frustum
upRight = ncpCenter + shiftDirection(fovVTan, fovVTan / aspect, R) * nearClipPlane;
upLeft = ncpCenter + shiftDirection(-fovVTan, fovVTan / aspect, R) * nearClipPlane;
downRight = ncpCenter + shiftDirection(fovVTan, -fovVTan / aspect, R) * nearClipPlane;
downLeft = ncpCenter + shiftDirection(-fovVTan, -fovVTan / aspect, R) * nearClipPlane;

length = 60;                % Длина стороны плоскости пола
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

% Если рёбра frustum пересекают плоскость пола, то строим пересечение на
% графике
if (checkUpRight == 1) && (checkUpLeft == 1) && (checkDownRight == 1) && (checkDownLeft == 1)
    intersectPos = [posUpRight; posUpLeft; posDownLeft; posDownRight];
    f = fill3(intersectPos(:,1),intersectPos(:,2),intersectPos(:,3),'green');
    hold on
end

% Построение frustum
P = [upRight; upLeft; downLeft; downRight; camPos];
ind = [1 2 5]; patch(P(ind, 1), P(ind, 2), P(ind, 3), 'red');
hold on
ind = [2 3 5]; patch(P(ind, 1), P(ind, 2), P(ind, 3), 'red');
ind = [3 4 5]; patch(P(ind, 1), P(ind, 2), P(ind, 3), 'red');
ind = [4 1 5]; patch(P(ind, 1), P(ind, 2), P(ind, 3), 'red');

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