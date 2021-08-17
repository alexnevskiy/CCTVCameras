function TR = getTriangulatedFrustum(top,upRight,upLeft,downRight,downLeft)
    P = [   top;upRight;upLeft;downRight;downLeft];
    
    T = [   1   2   4;
            1   4   5;
            1   5   3;
            1   3   2;
            3   2   5;
            5   2   4];
        
    TR = triangulation(T,P);
end