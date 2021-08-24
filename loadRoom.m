function [roomH,gridStep,wallsPts,doors,windows,camPos] = loadRoom(fileName)
    filetext  = fileread(fileName);
    expr = '//.*\n';
    filetext = erase(filetext,regexpPattern(expr,'DotExceptNewline',true));

    roomH = sscanf(filetext,'height %f');
    expr = 'height\s+\d+\.*\d*';
    filetext = erase(filetext,regexpPattern(expr,'DotExceptNewline',true));
    
    gridStep = sscanf(filetext,'\ngrid step %f');
    expr = 'grid step\s+\d+\.*\d*';
    filetext = erase(filetext,regexpPattern(expr,'DotExceptNewline',true));

    newStr = extractBetween(filetext,"wall coordinates","doors");
    wallsPts = sscanf(newStr{1,1},'%f %f\n',[2 Inf]);
    wallsPts = wallsPts';
    filetext = eraseBetween(filetext,"wall coordinates","doors");
    expr = 'wall coordinates';
    filetext = erase(filetext,regexpPattern(expr));
    
    newStr = extractBetween(filetext,"doors","windows");
    pattern = strcat('(?<WallNumber>\d+)\s+(?<DistanceToDoor>\d+\.*\d*)\s+(?<DoorWidth>\d+\.*\d*)\s+',...
        '(?<DoorHeight>\d+\.*\d*)\s+(?<Doorhandle>\w+)\s+(?<WhereOpen>\w+)[\r\n]');
    doors = regexp(newStr, pattern, 'names');
    doors = doors{1,1}.';
    [mDoors,~] = size(doors);
    for i = 1:mDoors
        doors(i).WallNumber = str2double(doors(i).WallNumber);
        doors(i).DistanceToDoor = str2double(doors(i).DistanceToDoor);
        doors(i).DoorWidth = str2double(doors(i).DoorWidth);
        doors(i).DoorHeight = str2double(doors(i).DoorHeight);
    end
    filetext = eraseBetween(filetext,"doors","windows");
    expr = 'doors';
    filetext = erase(filetext,regexpPattern(expr));
    filetext = strtrim(filetext);
    
    newStr = extractBetween(filetext,"windows","camera position");
    pattern = strcat('(?<WallNumber>\d+)\s+(?<DistanceToWindow>\d+\.*\d*)\s+(?<WindowWidth>\d+\.*\d*)\s+',...
        '(?<WindowHeight>\d+\.*\d*)\s+(?<FloorDistance>\d+\.*\d*)[\r\n]');
    windows = regexp(newStr, pattern, 'names');
    windows = windows{1,1}.';
    [mWindows,~] = size(windows);
    for i = 1:mWindows
        windows(i).WallNumber = str2double(windows(i).WallNumber);
        windows(i).DistanceToWindow = str2double(windows(i).DistanceToWindow);
        windows(i).WindowWidth = str2double(windows(i).WindowWidth);
        windows(i).WindowHeight = str2double(windows(i).WindowHeight);
        windows(i).FloorDistance = str2double(windows(i).FloorDistance);
    end
    filetext = eraseBetween(filetext,"windows","camera position");
    expr = 'windows';
    filetext = erase(filetext,regexpPattern(expr));
    filetext = strtrim(filetext);

    camPos = sscanf(filetext,'camera position %f %f %f',[1 3]);
end