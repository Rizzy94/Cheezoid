function [nxt,arrived,lost,offPath] = pathFollow(nxt, to, goalPos)
    % arrived is "at goal",lost means estimate is shit, offPath means good
    % estimate but too far off planned route, so replan
    map = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]; %default map
    lost = 0;
    from = nxt.pos;
    %work out the distance and angle between from and to
    xDif = to(1)-from(1);
    yDif = to(2)-from(2);
    % moveDist = sqrt((xDif^2) + (yDif^2))
    moveDist = distance(nxt.pos, to)
    angle = atan2(yDif,xDif)
            
    idealBot = BotSim(map);
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
    
    if turnAngle ~= 0 
        nxt.turn(turnAngle)
    end
    if moveDist ~= 0
     nxt.move(moveDist)
    end
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
         CheckBots(i) = BotSim(map);
         CheckBots(i).setBotPos(from); % ideal bot or current estimate?
         CheckBots(i).setBotAng(angle);
         CheckBots(i).setScanConfig(CheckBots.getScanConfig(30));
    end

    %set them to positions corresponding to movement and turn angle
    turnNoise = 0.3*turnAngle;
    moveNoise = 0.3*moveDist;
    
    oScans = zeros(4,checkBotsNmum);
    
    for i = 1:checkBotsNum
        CheckBots(i).turn(normrnd(turnAngle,turnNoise));
        CheckBots(i).move(normrnd(moveDist,moveNoise));
        % take scans and collect orthoscans
        scan = CheckBots(i).ultraScan();
        [~, oScans(:,i)] = orthoScans(scan);
    end

    %perform a scan, take the orthogonals compare to orthogonal particles
    
    nxtScan = nxt.rotScan(30);
    [~,nxtOrthoScan] = orthoScans(nxtScan);
    
    diff = zeros(4,checkBotsNum);
    prob = zeros(1,checkBotsNum);

    for i = 1:checkBotsNum
        diff(:,i) = abs(nxtOrthoScan - oScans(:,i));
        prob(1,i) = prod(normpdf(diff(:,i),0,4));       
    end
    
    [~,I] = max(prob);
    
    if sum(diff(:,I)) < 20
        nxt.pos = CheckBots(I).getBotPos();
        nxt.ang = CheckBots(I).getBotAng();
    else
        disp('NXT is lost, relocalise')
        lost = 1;
        return
    end
    
    if sum(nxt.pos - to) > 5
        lost = 0;
        arrived = 0;
        offPath = 1;
        return
    end

    %update nxt.pos & nxt.ang
    %check if arrived.
    arrived = 0;
    if distance(nxt.pos, goalPos) < 5
        arrived = 1;
    end
end