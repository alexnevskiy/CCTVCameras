function TR = getWall(startPos,endPos,height,floorDistance)
    if nargin == 3
        floorDistance = 0;
    end
    
    P = [   startPos(1,1)    startPos(1,2)    floorDistance;...
            startPos(1,1)    startPos(1,2)    floorDistance + height;...
            endPos(1,1)      endPos(1,2)      floorDistance + height;...
            endPos(1,1)      endPos(1,2)      floorDistance];
    
    T = [   1   2   3;
            1   3   4];
        
    TR = triangulation(T,P);
end