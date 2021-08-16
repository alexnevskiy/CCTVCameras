function TRInter = planeObjectIntersection(TR,X,V0)
    %% Разрезание объекта, нахождение точек пересечения с плоскостью и создание переменных
    % При помощи написанной функции cutObject() получаем нужные для
    % дальнейшей работы массивы, затем находим пересечение обрезанной
    % фигуры с плоскостью при помощи функции intersectPlaneSurf(),
    % скачанной из интернета. Так как функция возвращает матрицу, где row -
    % это координаты, а column - порядковый номер координаты, то
    % транспонируем её. Далее создаём переменные.
    
    [points,faces,boundaryPoly,pointsAbovePlain] = cutObject(TR,X,V0);

    Surface1.vertices = TR.Points;
    Surface1.faces = TR.ConnectivityList;

    lin = intersectPlaneSurf(Surface1,V0,X);
    lin = cat(2,lin{:});
    intersections = lin.';
    
    [mBoundary,~] = size(boundaryPoly);
    [mPoints,~] = size(points);
    [mIntersections,~] = size(intersections);
    ind = mPoints + 1;
    
    points = [points; zeros(mIntersections, 3)];
    pointsAbovePlain = [pointsAbovePlain; true(mIntersections, 1)];
    
    %% Поиск точек пересечения для самих полигонов
    % Рассматриваем каждый граничащий полигон и ищем точки, в которых он
    % пересекает плоскость. Для нахождения пересечения ребра полигона с
    % точкой используется функция collinear() из интернета, которая с
    % заданной погрешностью вычисляет, лежат ли все поданые на вход точки
    % на одной прямой или нет. Далее добавляем точки в массив points и
    % получаем их идентификаторы. Если вторая точка пересечения равна NaN,
    % то значит лишь одна точка лежит на плоскости, поэтому для создания
    % полигона даём ей тот же самый порядковый номер, что и первой точке
    % (небольшой такой костыль :)).
    
    for i = 1:mBoundary
        first = points(boundaryPoly(i,1),:);
        second = points(boundaryPoly(i,2),:);
        third = points(boundaryPoly(i,3),:);

        inter1 = NaN;
        inter2 = NaN;
        for j = 1:mIntersections
            if collinear([first; intersections(j,:); second],1e-10)
                if isnan(inter1)
                    inter1 = intersections(j,:);
                else
                    inter2 = intersections(j,:);
                    break
                end
            elseif collinear([first; intersections(j,:); third],1e-10)
                if isnan(inter1)
                    inter1 = intersections(j,:);
                else
                    inter2 = intersections(j,:);
                    break
                end
            elseif collinear([second; intersections(j,:); third],1e-10)
                if isnan(inter1)
                    inter1 = intersections(j,:);
                else
                    inter2 = intersections(j,:);
                    break
                end
            end
        end
        
        if isempty(find(ismember(points,inter1,'rows'), 1))
            inter1Ind = ind;
            points(inter1Ind,:) = inter1;
            ind = ind + 1;
        else
            [inter1Ind,~] = find(ismember(points,inter1,'rows'), 1);
        end
        
        if isnan(inter2)
            inter2Ind = inter1Ind;
        else
            if isempty(find(ismember(points,inter2,'rows'), 1))
                inter2Ind = ind;
                points(inter2Ind,:) = inter2;
                ind = ind + 1;
            else
                [inter2Ind,~] = find(ismember(points,inter2,'rows'), 1);
            end
        end
        
        %% Создание полигонов с новыми точками
        % Хитрой системой ифов создаём правильную триангуляцию для
        % получившихся пересечений: это либо четырёхугольник, который нужно
        % поделить на два треугольника (полигона), либо треугольник,
        % который сразу становится полигоном, либо точка, которая
        % становится тоже полигоном (тот самый костыль).

        firstAbove = pointsAbovePlain(boundaryPoly(i,1),1);
        secondAbove = pointsAbovePlain(boundaryPoly(i,2),1);
        thirdAbove = pointsAbovePlain(boundaryPoly(i,3),1);
        
        if firstAbove && secondAbove
            quadrilateral = [first;second;inter1;inter2];
            indQ = [boundaryPoly(i,1);boundaryPoly(i,2);inter1Ind;inter2Ind];
            quadrilateralMean = mean(quadrilateral,1);
            [~,~,V] = svd(quadrilateral - quadrilateralMean,0);
            quadrilateral2D = quadrilateral*V(:,1:2);
            order = convhull(quadrilateral2D(:,1),quadrilateral2D(:,2));
            
            faces = [faces; ...
                        [indQ(order(1)) indQ(order(2)) indQ(order(3))];...
                        [indQ(order(1)) indQ(order(3)) indQ(order(4))]];
        elseif firstAbove && thirdAbove
            quadrilateral = [first;third;inter1;inter2];
            indQ = [boundaryPoly(i,1);boundaryPoly(i,3);inter1Ind;inter2Ind];
            quadrilateralMean = mean(quadrilateral,1);
            [~,~,V] = svd(quadrilateral - quadrilateralMean,0);
            quadrilateral2D = quadrilateral*V(:,1:2);
            order = convhull(quadrilateral2D(:,1),quadrilateral2D(:,2));
            
            faces = [faces; ...
                        [indQ(order(1)) indQ(order(2)) indQ(order(3))];...
                        [indQ(order(1)) indQ(order(3)) indQ(order(4))]];
        elseif secondAbove && thirdAbove
            quadrilateral = [second;third;inter1;inter2];
            indQ = [boundaryPoly(i,2);boundaryPoly(i,3);inter1Ind;inter2Ind];
            quadrilateralMean = mean(quadrilateral,1);
            [~,~,V] = svd(quadrilateral - quadrilateralMean,0);
            quadrilateral2D = quadrilateral*V(:,1:2);
            order = convhull(quadrilateral2D(:,1),quadrilateral2D(:,2));
            
            faces = [faces; ...
                        [indQ(order(1)) indQ(order(2)) indQ(order(3))];...
                        [indQ(order(1)) indQ(order(3)) indQ(order(4))]];
        elseif firstAbove
            faces = [faces; [boundaryPoly(i,1) inter1Ind inter2Ind]];
        elseif secondAbove
            faces = [faces; [boundaryPoly(i,2) inter1Ind inter2Ind]];
        else
            faces = [faces; [boundaryPoly(i,3) inter1Ind inter2Ind]];
        end
    end
    
    %% Новая индексация массива
    % Тот же самый принцип работы, что и в 4 пункте функции cutObject()
    
    [mPoints,~] = size(points);
    newIndexes = zeros(mPoints,1);
    [falseCount,~] = find(ismember(pointsAbovePlain,false));
    [falseCount,~] = size(falseCount);
    j = 1;
    for i = 1:mPoints-falseCount
        while ~pointsAbovePlain(j,1)
            j = j + 1;
        end
        newIndexes(j,1) = i;
        j = j + 1;
    end
    
    %% Замена старых вершин в полигонах на новые
    % Аналогично 5 пункту функции cutObject()
    
    [mFaces,~] = size(faces);
    for i = mFaces:-1:1
        if      (pointsAbovePlain(faces(i,1),1) == false) || ...
                (pointsAbovePlain(faces(i,2),1) == false) || ...
                (pointsAbovePlain(faces(i,3),1) == false)
            faces(i,:) = [];
        end
    end
    
    [mFaces,~] = size(faces);
    for i = 1:mFaces
        first = newIndexes(faces(i,1),1);
        second = newIndexes(faces(i,2),1);
        third = newIndexes(faces(i,3),1);
        faces(i,:) = [first second third];
    end
    
    %% Удаление точек, которые находятся ниже плоскости
    % Аналогично 6 пункту функции cutObject()
    
    for i = mPoints:-1:1
        if pointsAbovePlain(i,1) == false
            points(i,:) = [];
        end
    end
    
    %% Получение объекта триангуляции
    TRInter = triangulation(faces, points);
end