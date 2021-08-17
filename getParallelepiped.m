function TR = getParallelepiped(width,depth,height,position)
    x = position(1);
    y = position(2);
    z = position(3);
    
    P = [   x+width     y+depth     z;...
            x           y+depth     z;...
            x           y+depth     z+height;...
            x+width     y+depth     z+height;...
            x+width     y           z+height;...
            x           y           z+height;...
            x           y           z;...
            x+width     y           z];
    
    T = [   2   4   1;
            4   6   5;
            3   7   6;
            8   6   7;
            1   5   8;
            2   8   7;
            2   3   4;
            4   3   6;
            3   2   7;
            8   5   6;
            1   4   5;
            2   1   8];
        
    TR = triangulation(T,P);
end