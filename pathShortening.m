function [shortPath,shortPathLength] = pathShortening(pathCoord,pathLength,map)

orientAngles = 30;
to = [];
checkBot = BotSim(map);
checkBot.setBotPos(pathCoord(1,:));
checkBot.setScanConfig(checkBot.generateScanConfig(orientAngles));

shortPath = [pathCoord(1,:)];

coordNum = 1;
if ~safePath(checkBot, pathCoord(1,:),  pathCoord(2,:)) %Say if this is one
    warning('Starting from too close to wall, please update pathShortening.m to cope')
end

while true
    from = shortPath(end,:); %good
    for i = coordNum+1:size(pathCoord,1)    
        if safePath(checkBot, from, pathCoord(i,:))
            to = pathCoord(i,:);
        else
            coordNum = i - 1;
            break
        end
    end
    shortPath = [shortPath; to];
    
    %stop once the goal node had been added
    if to == pathCoord(end,:)
        break
    end
end

shortPath
figure(2)
checkBot.drawMap();
checkBot.drawBot(3);
scatter(shortPath(:,1),shortPath(:,2))
shortPathLength = size(shortPath,1);

end