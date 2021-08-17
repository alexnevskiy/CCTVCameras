function boundaries = findBoundaries(TR)
    [m,~] = size(TR.Points);
    for i = 1:m
        if i == 1
            minX = TR.Points(i,1);
            maxX = TR.Points(i,1);
            minY = TR.Points(i,2);
            maxY = TR.Points(i,2);
        end
        
        if minX > TR.Points(i,1)
            minX = TR.Points(i,1);
        end
        
        if minY > TR.Points(i,2)
            minY = TR.Points(i,2);
        end
        
        if maxX < TR.Points(i,1)
            maxX = TR.Points(i,1);
        end
        
        if maxY < TR.Points(i,2)
            maxY = TR.Points(i,2);
        end
    end
    
    boundaries = [  minX minY 0; 
                    minX maxY 0; 
                    maxX maxY 0; 
                    maxX minY 0];
end