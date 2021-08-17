function TRPlane = planeProjection(TR,X,V0,T,camPos,P1,P2)
    [m,~] = size(TR.Points);
    planePoints = zeros(m,3);
    
    for i = 1:m
        [I,check] = planeLineIntersect(X,V0,TR.Points(i,:),camPos);
        if check == 1
            planePoints(i,:) = I;
        else
            if ((dot(I - camPos, T) >= 0) && (check == 3))
                planePoints(i,:) = I;
            else
                P3 = [P1(1,1), P1(1,2), P1(1,3) + 1];
                normal = cross(P1-P2, P1-P3);
                [I,~] = planeLineIntersect(normal,P1,TR.Points(i,:),camPos);
                I(1,3) = V0(1,3);
                planePoints(i,:) = I;
            end
        end
    end
    
    TRPlane = triangulation(TR.ConnectivityList,planePoints);
end