function [roomH,gridStep,wallsPts,camPos] = loadRoom(fileName)
    filetext  = fileread(fileName);
    expr = '//.*\n';
    filetext = erase(filetext,regexpPattern(expr,'DotExceptNewline',true));

    roomH = sscanf(filetext,'height %f');
    expr = 'height\s+\d+\.*\d*';
    filetext = erase(filetext,regexpPattern(expr,'DotExceptNewline',true));
    
    gridStep = sscanf(filetext,'\ngrid step %f');
    expr = 'grid step\s+\d+\.*\d*';
    filetext = erase(filetext,regexpPattern(expr,'DotExceptNewline',true));

    wallsPts = sscanf(filetext,'%f %f\n',[2 Inf]);
    wallsPts = wallsPts';
    filetext = erase(filetext,wildcardPattern + 'camera position ');

    camPos = sscanf(filetext,'%f %f %f',[1 3]);
end