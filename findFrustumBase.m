function [upRight,upLeft,downRight,downLeft] = findFrustumBase(cpCenter,fovHTan,fovVTan,R,clipPlane)
    upRight = cpCenter + shiftDirection(fovHTan, -fovVTan, R) * clipPlane;
    upLeft = cpCenter + shiftDirection(-fovHTan, -fovVTan, R) * clipPlane; 
    downRight = cpCenter + shiftDirection(fovHTan, fovVTan, R) * clipPlane; 
    downLeft = cpCenter + shiftDirection(-fovHTan, fovVTan, R) * clipPlane;
end