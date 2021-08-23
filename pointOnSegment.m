function check = pointOnSegment(p1,p2,point,tol)
    x1 = p1(1,1);
    y1 = p1(1,2);
    z1 = p1(1,3);
    
    x2 = p2(1,1);
    y2 = p2(1,2);
    z2 = p2(1,3);
    
    x = point(1,1);
    y = point(1,2);
    z = point(1,3);
    
    AB = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1));
    AP = sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1)+(z-z1)*(z-z1));
    PB = sqrt((x2-x)*(x2-x)+(y2-y)*(y2-y)+(z2-z)*(z2-z));
    
    if abs(AB - (AP + PB)) < tol
        check = true;
    else
        check = false;
    end
end