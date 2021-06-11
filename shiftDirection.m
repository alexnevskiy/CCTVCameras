function F = shiftDirection(x, z, R)
    X = [x z 0];
    F = X * R;
end