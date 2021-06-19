function conjunction = planesConjunction(planeInterLimit, planeInterFloor) 
    planeInterLimit(:,end) = [];
    planeInterFloor(:,end) = [];
    
    polyLimit = polyshape(planeInterLimit(1:end,:));
    polyFloor = polyshape(planeInterFloor(1:end,:));
    
    conjunction = intersect(polyLimit, polyFloor);

    conjunction = conjunction.Vertices;
    conjunction = [conjunction zeros(size(conjunction,1),1)];
end