function [nxt,arrived,lost] = pathFollow(nxt, to)
    map = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]; %default map
    lost = 0;
    from = nxt.pos;
    %work out the distance and angle between from and to
    xDif = to(1)-from(1);
    yDif = to(2)-from(2);
    % moveDist = sqrt((xDif^2) + (yDif^2))
    moveDist = distance(nxt.pos, to);
    angle = atan2(yDif,xDif);
        
    
    idealBot = Botsim(map);
    idealBot.setBotPos(nxt.pos);
    idealBot.setBotAng(nxt.ang);
    idealBot.drawMap();
    idealBot.drawBot(5);

    %work out how much to turn robot
    turnAngle = angle - nxt.ang;
    if turnAngle > pi
        turnAngle = -(2*pi - turnAngle);
    elseif turnAngle < -pi
        turnAngle = turnAngle + 2*pi;
    end

    %move the robot
    nxt.turn(turnAngle)
    nxt.move(moveDist)
    %update where the robot thinks it is
    nxt.pos = to;
    nxt.ang = angle;
    
    idealBot.setBotPos(nxt.pos);
    idealBot.setBotAng(nxt.ang);
    idealBot.drawBot(5);


    %check particles
    
    %initalise the particles
    checkBotsNum = 500;
    CheckBots(checkBotsNum,1) = BotSim;
    for i = 1:checkBotsNum
         CheckBots(i) = BotSim(modifiedMap);
         CheckBots(i).setScanConfig(CheckBots(i).generateScanConfig(orientAngles))
         CheckBots(i).setBotPos(from); % ideal bot or current estimate?
         CheckBots(i).setBotAng(CurrentBotEstimate.getBotAng());
    end

    %set them to positions corresponding to movement and turn angle
    turnNoise = 0.3*turnAngle;
    moveNoise = 0.3*moveDist;
    
    for i = 1:checkBotsNum
        CheckBots(i).turn(normrnd(turnAngle,turnNoise));
        CheckBots(i).move(normrnd(moveDist,moveNoise));
        
    end

    %perform a scan, take the orthogonals compare to orthogonal particles

    %update nxt.pos & nxt.ang

    %check if arrived.
    arrived = 0;
    if distance(nxt.pos, nxt.goalPos) < 5
        arrived = 1
    end
end