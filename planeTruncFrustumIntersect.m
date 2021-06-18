function inter = planeTruncFrustumIntersect(X,V0,nearUpRight,nearUpLeft,nearDownRight,nearDownLeft,...
    farUpRight,farUpLeft,farDownRight,farDownLeft)
    [interCheckLeftFar, interLeftFar] = IntersectPlaneTriangle(X, V0, nearUpLeft, farDownLeft, farUpLeft);
    [interCheckLeftNear, interLeftNear] = IntersectPlaneTriangle(X, V0, nearUpLeft, nearDownLeft, farDownLeft);
    
    [interCheckUpFar, interUpFar] = IntersectPlaneTriangle(X, V0, nearUpLeft, farUpLeft, farUpRight);
    [interCheckUpNear, interUpNear] = IntersectPlaneTriangle(X, V0, nearUpLeft, nearUpRight, farUpRight);
    
    [interCheckRightFar, interRightFar] = IntersectPlaneTriangle(X, V0, nearUpRight, farUpRight, farDownRight);
    [interCheckRightNear, interRightNear] = IntersectPlaneTriangle(X, V0, nearUpRight, nearDownRight, farDownRight);
    
    [interCheckDownFar, interDownFar] = IntersectPlaneTriangle(X, V0, nearDownRight, farDownRight, farDownLeft);
    [interCheckDownNear, interDownNear] = IntersectPlaneTriangle(X, V0, nearDownRight, nearDownLeft, farDownLeft);
    
    [interCheckLeft, interLeft] = deleteCommon(interLeftFar.', interCheckLeftFar, interLeftNear.', interCheckLeftNear);
    [interCheckUp, interUp] = deleteCommon(interUpFar.', interCheckUpFar, interUpNear.', interCheckUpNear);
    [interCheckRight, interRight] = deleteCommon(interRightFar.', interCheckRightFar, interRightNear.', interCheckRightNear);
    [interCheckDown, interDown] = deleteCommon(interDownFar.', interCheckDownFar, interDownNear.', interCheckDownNear);
    
    arr = [interLeft; interUp; interRight; interDown];
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

function [interCheck, interCommon] = deleteCommon(interFar, interCheckFar, interNear, interCheckNear)
    if interCheckFar == 1
        interFar(2,:) = [];
    elseif interCheckFar == 0
        interFar(2,:) = [];
        interFar(1,:) = [];
    end

    if interCheckNear == 1
        interNear(2,:) = [];
    elseif interCheckNear == 0
        interNear(2,:) = [];
        interNear(1,:) = [];
    end

    [mFar, ~] = size(interFar);
    [mNear, ~] = size(interNear);

    if ((mFar ~= 0) || (mNear ~= 0))
        for i = mFar:-1:1
            pointFar = interFar(i,:);
            for j = mNear:-1:1
                if interNear(j,:) == pointFar
                    interNear(j,:) = [];
                    interFar(i,:) = [];
                    mNear = mNear - 1;
                end
            end
        end
    end

    interCheck = 2;
    interCommon = [interFar; interNear];
    [m, ~] = size(interCommon);
    if m == 1
        interCommon(2,:) = [0 0 0];
        interCheck = 1;
    elseif m == 0
        interCommon(1,:) = [0 0 0];
        interCommon(2,:) = [0 0 0];
        interCheck = 0;
    end
end