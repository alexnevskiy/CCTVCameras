function unPoly = unionPolygons(TR)
    [m,~] = size(TR.ConnectivityList);
    for i = 1:m
        if i == 1
            unPoly = polyshape(TR.Points(TR.ConnectivityList(1,:),1:2));
        else
            p = polyshape(TR.Points(TR.ConnectivityList(i,:),1:2));
            unPoly = union(unPoly, p);
        end
    end
end
