function TR = getWall(startPos,endPos,height)
    P = [   startPos(1,1)    startPos(1,2)    0;...
            startPos(1,1)    startPos(1,2)    height;...
            endPos(1,1)      endPos(1,2)      height;...
            endPos(1,1)      endPos(1,2)      0];
    
    T = [   1   2   3;
            1   3   4];
        
    TR = triangulation(T,P);
end