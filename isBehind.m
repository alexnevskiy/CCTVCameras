function behind = isBehind(TR,X,V0,mode)
    [m,~] = size(TR.Points);
    behind = false;
    counter = 0;
    
    for i = 1:m
        if dot(TR.Points(i,:) - V0, X) < 0
            if strcmp(mode,'any')
                behind = true;
                return
            elseif strcmp(mode,'full')
                counter = counter + 1;
            end
        end
    end
    
    if counter == m
        behind = true;
    end
end