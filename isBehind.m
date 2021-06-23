function behind = isBehind(TR,T,camPos)
    [m,~] = size(TR.Points);
    behind = false;
    for i = 1:m
        if dot(TR.Points(i,:) - camPos, T) < 0
            behind = true;
            return
        end
    end
end