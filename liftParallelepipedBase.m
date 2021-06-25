function liftTR = liftParallelepipedBase(TR,V0)
    points = TR.Points;
    [m,~] = size(points);
    for i = 1:m
        if points(i,3) < V0(1,3)
            points(i,3) = V0(1,3);
        end
    end
    
    liftTR = triangulation(TR.ConnectivityList,points);
end