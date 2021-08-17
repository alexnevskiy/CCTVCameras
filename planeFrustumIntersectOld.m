function [posUpRight,checkUpRight,posUpLeft,checkUpLeft,posDownRight,checkDownRight,...
    posDownLeft,checkDownLeft] = planeFrustumIntersectOld(X,V0,camPos,upRight,upLeft,downRight,downLeft)
    [posUpRight, checkUpRight] = planeLineIntersect(X, V0, camPos, upRight);
    [posUpLeft, checkUpLeft] = planeLineIntersect(X, V0, camPos, upLeft);
    [posDownRight, checkDownRight] = planeLineIntersect(X, V0, camPos, downRight);
    [posDownLeft, checkDownLeft] = planeLineIntersect(X, V0, camPos, downLeft);
end