function [inter,posUpRight,checkUpRight,posUpLeft,checkUpLeft] ...
    = plotPlaneFrustumIntersectOld(posUpRight,checkUpRight,posUpLeft,checkUpLeft,posDownRight,checkDownRight,...
    posDownLeft,checkDownLeft,X,V0,downRight,upRight,downLeft,upLeft,color)
    if (checkUpRight == 3) && (checkUpLeft == 3) ...
            && (checkDownRight == 1) && (checkDownLeft == 1)
        [posUpRight, checkUpRight] = planeLineIntersect(X, V0, downRight, upRight);
        [posUpLeft, checkUpLeft] = planeLineIntersect(X, V0, downLeft, upLeft); 
    end
    
    if (checkUpRight == 1) && (checkUpLeft == 1) ...
            && (checkDownRight == 1) && (checkDownLeft == 1)
        intersectPos = [posUpRight; posUpLeft; posDownLeft; posDownRight];
        inter = fill3(intersectPos(:,1),intersectPos(:,2),intersectPos(:,3),color);
        hold on
    else
        inter = fill3([0 0 0], [0 0 0], [0 0 0], color);
        hold on
    end
end