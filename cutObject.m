function [points,faces,boundaryPoly,pointsAbovePlain] = cutObject(TR,X,V0)
    %% Создание переменных
    % Данная функция обрезает у подаваемого на вход триангулированного
    % объекта полигоны, которые находятся ниже плоскости. Возвращает:
    %
    % points - набор точек в обрезанной фигуре
    % faces - набор вершин полигонов
    % boundaryPoly - полигоны, которые граничат с плоскостью, то есть хотя
    % бы одна точка полигона лежит ниже плоскости
    % pointsAbovePlain - битовый массив, который хранит boolean для каждой
    % точки: true - если выше плоскости, false - ниже
    
    [mPoints,~] = size(TR.Points);
    pointsAbovePlain = false(mPoints,1);
    points = TR.Points;
    
    [mFaces,~] = size(TR.ConnectivityList);
    faces = TR.ConnectivityList;
    
    boundaryPoly = zeros(mFaces,3);
    
    %% 1
    % Битовый массив, в котором хранятся используемые/неиспользуемые точки
    % относительно плоскости.
    
    for i = 1:mPoints
        if dot(TR.Points(i,:) - V0, X) >= 0
            pointsAbovePlain(i,1) = true;
        end  
    end
    
    %% 2
    % Выбрасываем те полигоны (faces), в которых все 3 точки находятся ниже
    % плоскости, а также добавляем те полигоны, что находятся на границе с
    % плоскостью.
    
    j = 1;
    for i = mFaces:-1:1
        if      (pointsAbovePlain(faces(i,1),1) == false) && ...
                (pointsAbovePlain(faces(i,2),1) == false) && ...
                (pointsAbovePlain(faces(i,3),1) == false)
            faces(i,:) = [];
        elseif  (pointsAbovePlain(faces(i,1),1) == false) || ...
                (pointsAbovePlain(faces(i,2),1) == false) || ...
                (pointsAbovePlain(faces(i,3),1) == false)
            boundaryPoly(j,:) = faces(i,:);
            j = j + 1;
        end
    end
    
    for i = j:mFaces
        boundaryPoly(j,:) = [];
    end
    
    %% 3
    % Создаём битовый массив со всеми false, итерируемся по укороченному
    % массиву полигонов и отмечаем true те точки в новом битовом массиве,
    % что хоть раз используются, тем самым отсекая полностью неиспользуемые
    % точки. Также подсчитываем количество неиспользуемых точек.
    
    usedPoints = false(mPoints,1);
    [mFaces,~] = size(faces);
    
    for i = 1:mFaces
        usedPoints(faces(i,1),1) = true;
        usedPoints(faces(i,2),1) = true;
        usedPoints(faces(i,3),1) = true;
    end
    
    falseCount = 0;
    
    for i = 1:mPoints
        if usedPoints(i,1) == false
            falseCount = falseCount + 1;
        end
    end
    
    %% 4
    % Создаём новый массив для новых индексов той же размерности, что и
    % массив со всеми точками. Итерируемся по нему, где пределом будет
    % количество всех вершин (mPoints) минус количество false вершин
    % (falseCount). Если вершина true, то записываем в неё новый индекс,
    % если false, то пропускам её и рассматриваем следующую. Для этого
    % здесь используется счётчик j, который подсчитывает нынешнее положение 
    % в массиве.
    
    newIndexes = zeros(mPoints,1);
    j = 1;
    for i = 1:mPoints-falseCount
        while ~usedPoints(j,1)
            j = j + 1;
        end
        newIndexes(j,1) = i;
        j = j + 1;
    end
    
    %% 5
    % Заменяем старые индексы вершин на новые для все полигонов и тех, что
    % граничат с плоскостью.
    
    for i = 1:mFaces
        first = newIndexes(faces(i,1),1);
        second = newIndexes(faces(i,2),1);
        third = newIndexes(faces(i,3),1);
        faces(i,:) = [first second third];
    end
    
    [mBoundary,~] = size(boundaryPoly);
    for i = 1:mBoundary
        first = newIndexes(boundaryPoly(i,1),1);
        second = newIndexes(boundaryPoly(i,2),1);
        third = newIndexes(boundaryPoly(i,3),1);
        boundaryPoly(i,:) = [first second third];
    end
    
    %% 6
    % Удаляем из массивов с вершинами и вершинами выше плоскости те, что
    % помечены false в массиве с используемыми точками.
    
    for i = mPoints:-1:1
        if usedPoints(i,1) == false
            points(i,:) = [];
            pointsAbovePlain(i,:) = [];
        end
    end
end