function [arrived,lost] = toFollow(nxt, to)
    
    orientAngles = 100; % I appreciate that this will probably end up in the robot class but for now
    from = nxt.pos;
    %work out the distance and angle between from and to
    xDif = to(1)-from(1);
    yDif = to(2)-from(2);
    % moveDist = sqrt((xDif^2) + (yDif^2))
    moveDist = distance(nxt.pos, to);
    angle = atan2(yDif,xDif);

    %work out how much to turn robot
    turnAngle = angle - nxt.ang
    if turnAngle > pi
        turnAngle = -(2*pi - turnAngle)
    elseif turnAngle < -pi
        turnAngle = turnAngle + 2*pi
    end

    %move the robot
    nxt.turn(turnAngle)
    nxt.move(C)
    %update where the robot thinks it is
    nxt.pos = to;
    nxt.ang = angle;

    %check particles
    
    %initalise the particles
    checkBotsNum = 500;
    CheckBots(checkBotsNum,1) = BotSim;
    for i = 1:checkBotsNum
         CheckBots(i) = BotSim(modifiedMap);
         CheckBots(i).setScanConfig(CheckBots(i).generateScanConfig(orientAngles)) % MAY NEED A FIDDLE WITH NEW SCANNING
         CheckBots(i).setBotPos(from); % ideal bot or current estimate?
         CheckBots(i).setBotAng(angle);
    end

    %set them to positions corresponding to movement and turn angle
    turnNoise = 0.3*turnAngle;
    moveNoise = 0.3*moveDist;
    
    % turn and move the checkBots by the same amount as the robot, +noise
    for i = 1:checkBotsNum
        CheckBots(i).turn(normrnd(turnAngle,turnNoise));
        CheckBots(i).move(normrnd(moveDist,moveNoise));        
    end

    %perform a scan, take the orthogonals compare to orthogonal particles
     
    % HAVEN'T ACTUALLY TAKEN SCANS BECAUSE I DUNNO HOW THEY'RE GOING TO
    % WORK
    
    checkBotScans = zeros(checkBotsNum,4);  % all checkbot scan values
    scanDiff = zeros(checkBotsNum,4);       % difference between the scans taken by the robot and all of the checkBot scans
    scanProb = zeros(checkBotsNum,1);       %probability of each checkbot scan 
    
    for j = 1:checkBotsNum
         if CheckBots(j).pointInsideMap(CheckBots(j).getBotPos()) == 1
             [checkBotScans(j,:),~] = CheckBots(j).ultraScan();             % NEEDS THE NEW SCAN SHIZZ
             for k = 1:length(botScan2)                                     % to stop 255 readings messing with probabilities, hopefully
                if botScan2(k,1) == 255
                   scanDiff(j,k) = 0;
                else
                   scanDiff(j,k) = abs(checkBotScans(j,k) - botScan2(k,1));
                end
             end
             scanProb(j,1) = prod(normpdf(scanDiff(j,:),0,3));             % I imagine this is going to need changing to something which can handle a shit sensor. Poisson dist? Log normal?        
         else
             scanProb(j,1) = -1; % stops points outside map getting picked
         end
    end
    
    [~,I4] = sort(scanProb);
    %update nxt.pos & nxt.ang
    
    % IF BEST CHECKBOT SCAN IS SHITE, SAY WE'RE LOST. OTHERWISE ASSIGN THE
    % CHECKBOT'S POSITION AND ANGLE TO THE NXT
    if mean(checkBots(I4(checkBotsNum)).ultraScan() - nxt.rotScan()) < 5   % DOUBT IT'LL BE ROTSCAN ONCE SCAN IS UPDATED
        nxt.pos = CheckBots(I4(checkBotsNum)).getBotPos();
        nxt.ang = CheckBots(I4(checkBotsNum)).getBotAng();      % CheckBots(I4(checkBotsNum)) is the checkBot with the best scan
    else
        lost = 1        % GONNA NEED TO OUTPUT LOST FROM THE FUNCTION
        disp('Best estimate is not a good estimate, relocalise')
        return  % NGL I DON'T KNOW IF THIS SKIPS TO THE END OF A FUNCTION BUT THAT'S WHAT I WANT
    end
    
    arrived = 0;
    if distance(nxt.pos, nxt.goalPos) < 5       % GONNA NEED ANOTHER CHECK FOR JUST WHETHER ITS HIT THE NODE FOR THIS STEP, NOT JUST THE GOAL
        arrived = 1
    end
end