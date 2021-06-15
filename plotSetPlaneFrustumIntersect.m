function [inter,posUpRight,checkUpRight,posUpLeft,checkUpLeft] ...
    = plotSetPlaneFrustumIntersect(posUpRight,checkUpRight,posUpLeft,checkUpLeft,posDownRight,checkDownRight,...
    posDownLeft,checkDownLeft,X,V0,downRight,upRight,downLeft,upLeft,inter)
    if (checkUpRight == 3) && (checkUpLeft == 3) ...
        && (checkDownRight == 1) && (checkDownLeft == 1)
        [posUpRight, checkUpRight] = planeLineIntersect(X, V0, downRight, upRight);
        [posUpLeft, checkUpLeft] = planeLineIntersect(X, V0, downLeft, upLeft); 
    end

    if (checkUpRight == 1) && (checkUpLeft == 1) ...
        && (checkDownRight == 1) && (checkDownLeft == 1)
        intersectPos = [posUpRight; posUpLeft; posDownLeft; posDownRight];
        set(inter, 'XData', intersectPos(:,1), 'YData', intersectPos(:,2), 'ZData', intersectPos(:,3));
    else
        set(inter, 'XData', [0 0 0], 'YData', [0 0 0], 'ZData', [0 0 0]);
    end
end