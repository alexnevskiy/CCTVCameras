function behind = isBehind(TR,X,V0)
    [m,~] = size(TR.Points);
    behind = false;
    for i = 1:m
        if dot(TR.Points(i,:) - V0, X) < 0
            behind = true;
            return
        end
    end
end