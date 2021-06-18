function inter = planeFrustumIntersect(X,V0,camPos,upRight,upLeft,downRight,downLeft)
    [interCheckLeft, interLeft] = IntersectPlaneTriangle(X, V0, camPos, downLeft, upLeft);
    [interCheckUp, interUp] = IntersectPlaneTriangle(X, V0, camPos, upLeft, upRight);
    [interCheckRight, interRight] = IntersectPlaneTriangle(X, V0, camPos, upRight, downRight);
    [interCheckDown, interDown] = IntersectPlaneTriangle(X, V0, camPos, downRight, downLeft);
    
    arr = [interLeft.'; interUp.'; interRight.'; interDown.'];
    arrCheck = [interCheckLeft interCheckUp interCheckRight interCheckDown];
    
    inter = zeros(4,3);
    ind = 1;
    
    for i = 1:2:8
        if arrCheck(floor(i/2+1)) == 1
            if (~ismember(arr(i,:), inter, 'rows'))
                inter(ind,:) = arr(i,:);
                ind = ind + 1;
            end
        elseif arrCheck(floor(i/2+1)) == 2
            if (~ismember(arr(i,:), inter, 'rows'))
                inter(ind,:) = arr(i,:);
                ind = ind + 1;
            end
            if (~ismember(arr(i+1,:), inter, 'rows'))
                inter(ind,:) = arr(i+1,:);
                ind = ind + 1;
            end
        end
    end
    
    for i = 4:-1:1
        if (inter(i,:) == [0 0 0])
            inter(i,:) = [];
        end 
    end
    
    [m, ~] = size(inter);
    if m ~= 0
        interC = mean(inter,1);
        P = inter - interC;
        [~,~,V] = svd(P,0);
        [~,is] = sort(atan2(P*V(:,1),P*V(:,2)));
        inter = inter(is([1:end 1]),:);
    end
end