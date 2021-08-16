function intersect = cameraWallIntersect(point,wallsPts,camW)
    [wallsCount,~] = size(wallsPts);
    for i = 1:wallsCount
        if i ~= wallsCount
            [~,D,~] = Perpendicular2Line(point,wallsPts(i,:),wallsPts(i + 1,:));
            intersect = camW / 2 >= D;
            if intersect
                return
            end  
        else
            [~,D,~] = Perpendicular2Line(point,wallsPts(i,:),wallsPts(1,:));
            intersect = camW / 2 >= D;
            if intersect
                return
            end  
        end
    end
end