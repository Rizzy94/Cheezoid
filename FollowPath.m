function [botPos,botAng,arrived,lost] = FollowPath(nxt, pathCoord,botPos,botAng,pathLength,samples,map,plotit)

CurrentBotEstimate = BotSim(map);
CurrentBotEstimate.setBotPos(botPos);
CurrentBotEstimate.setBotAng(botAng);
goalPos = pathCoord(pathLength,:);

orientAngles = samples;
modifiedMap = map;
arrived = 0;
lost = 0;

%% MOVE DOWN PATH SETUP
CurrentBotEstimate.setScanConfig(CurrentBotEstimate.generateScanConfig(orientAngles)) % make sure its in the right scan config, might not be necessary?

pathDistAng = zeros(pathLength-1,2); %set up vectors to contain step information
turns = zeros(pathLength-1,1);
%Bot.drawBot(5,'b');
%Setup idealbot
IdealBot = BotSim(modifiedMap); %initiate
IdealBot.setBotPos(pathCoord(1,:)); %set pos          % DO I STILL WANT TO FLIP THE PATH? I THINK NOT
IdealBot.setBotAng(CurrentBotEstimate.getBotAng()); %set ang
IdealBot.setScanConfig(IdealBot.generateScanConfig(orientAngles)) %set scan config
%setup checkbots
checkBotsNum = 500;
CheckBots(checkBotsNum,1) = BotSim;
for i = 1:checkBotsNum
     CheckBots(i) = BotSim(modifiedMap);
     CheckBots(i).setScanConfig(CheckBots(i).generateScanConfig(orientAngles))
     CheckBots(i).setBotPos(CurrentBotEstimate.getBotPos()); % ideal bot or current estimate?
     CheckBots(i).setBotAng(CurrentBotEstimate.getBotAng());
end
 % matrix and vectors for comparison of scans
 checkBotScans = zeros(checkBotsNum,orientAngles);
 scanDiff2 = zeros(checkBotsNum,orientAngles);
 scanProb2 = zeros(checkBotsNum,1);
 a = 0;

 %% ACTUAL MOTION BEGINS
 stepCount = 0;
 for i = 1:pathLength-1 % Motion Loop               - 1 because at the last position we have (give or take noise) arrived
     %calculate step turn and distance
     pathDistAng(i,1) = sqrt((pathCoord(i,1)-pathCoord(i+1,1))^2+(pathCoord(i,2)-pathCoord(i+1,2))^2);
     pathDistAng(i,2) = atan2((pathCoord(i+1,1)-pathCoord(i,1)),(pathCoord(i+1,2)-pathCoord(i,2)));      %angle is measured from positive x axis
     turns(i,1) = -CurrentBotEstimate.getBotAng() + pathDistAng(i,2)
     pathDistAng
     %make moves                                      STILL NEED TO NEGATE TURNS?
      % do something like after the turn if forward scan is less than the
     % movement then don't do it? Probs have to relocalise
     if plotit == 1
         figure(3)
         
         CurrentBotEstimate.drawMap();
         CurrentBotEstimate.drawBot(20,'r');
     end

     for j = 1:checkBotsNum
         CheckBots(j).setBotPos(CurrentBotEstimate.getBotPos()); % set the checkbots to where we think the bot currently is
         CheckBots(j).setBotAng(CurrentBotEstimate.getBotAng());
         CheckBots(j).turn(turns(i,1)+normrnd(0,0.2));           % turn checkbots (with noise)
         CheckBots(j).move(pathDistAng(i,1)+normrnd(0,pathDistAng(i,1)/3)); %move checkbots (again with noise)
         if plotit == 1
            %CheckBots(j).drawBot(10,'b');
         end
     end
     if plotit == 1
         pause(0.2);
     end

     IdealBot.setBotPos(flip(pathCoord(i+1,:)));
     IdealBot.setBotAng(pathDistAng(i,2));

     %scans
%      botSim.turn(turns(i,1)); 
     nxt.turn(turns(i,1));
     % DO TWO SEPARATE SCANS, BEFORE FORWARD MOVEMENT AND AFTER
%      [botScanMoveCheck,~] = botSim.ultraScan();
     botScanMoveCheck = nxt.rotScan(samples);

%      if botScanMoveCheck(1) < 14*pathDistAng(i,1)/10 % will edit predicted collision criteria to match bot
%          CurrentBotEstimate.turn(turns(i,1));
%          disp('Forward scan predicts collision; relocalise');
%          arrived = 0;
%          lost = 1;
%          break % if it detects a forward collision, presumably it is not on route, and so it will cancel pathfollowing and output arrived = 0                  
%      end

%      botSim.move(pathDistAng(i,1)); 
%      NXTMove(pathDistAng(i,1), cmPerDeg, robotMotorPow);
     nxt.move(pathDistAng(i,1))
%      [botScan2,~] = botSim.ultraScan();
%      [botScan2,~] = NXTUltraScan(samples, round(360/degPerDeg));
      botScan2 = nxt.rotScan(samples);

%       botScan2crap = nxt.rotScan(samples);
     [idealScan,~] = IdealBot.ultraScan();
     for j = 1:checkBotsNum
         if CheckBots(j).pointInsideMap(CheckBots(j).getBotPos()) == 1
             [checkBotScans(j,:),~] = CheckBots(j).ultraScan();
%              scanDiff2(j,:) = abs(checkBotScans(j,:) - botScan2');
             for k = 1:length(botScan2)                                     % new thing here to stop 255 readings messing with probabilities, hopefully
                if botScan2(k,1) == 255
                   scanDiff2(j,k) = 0;
                else
                   scanDiff2(j,k) = abs(checkBotScans(j,k) - botScan2(k,1));
                end
             end
             scanProb2(j,1) = prod(normpdf(scanDiff2(j,:),0,3));             % I imagine this is going to need changing to something which can handle a shit sensor. Poisson dist? Log normal?        
         else
             scanProb2(j,1) = -1; % stops points outside map getting picked
         end
     end
     % So now we have scan probabilities, we sort:
     [~,I4] = sort(scanProb2);
     % add in an if statement for incase the best one is shit
     CurrentBotEstimate.setBotPos(CheckBots(I4(checkBotsNum)).getBotPos());     % I4(checkBotsNum) is the last entry in the array, and is the position of the best scan in scanProb2
     CurrentBotEstimate.setBotAng(CheckBots(I4(checkBotsNum)).getBotAng());
     [estimateScan,~] = CurrentBotEstimate.ultraScan();
     %if sum(abs(CurrentBotEstimate.getBotPos() - IdealBot.getBotPos())) > 20 % if we're too far from the ideal bot, re plan
      
     
     estiDiff = estimateScan-botScan2;
     valid = 0;
     
     for k = 1:length(botScan2)
        if botScan2(k) == 255
            estiDiff(k) = 0;
        else
            estiDiff = abs(estimateScan(k)-botScan2(k));
            valid =+ 1;
        end
     end
     
%      if sum(estiDiff)/valid > 10
%          disp('Best estimate is bad so robot is considered lost, need to relocalise');
%          lost = 1;
%          arrived = 0;
%          break
% 
%      elseif sum(estiDiff)/valid <= 10 && sum(abs(CurrentBotEstimate.getBotPos() - IdealBot.getBotPos())) > 20
%          disp('Bot is off of planned route, but not lost. Replan route');
%          lost = 0;
%          arrived = 0;
%      end

     stepCount = stepCount + 1;
     if stepCount == pathLength - 1         % i.e. bot has 'arrived' or at least has done last step in its planned path
         break
     end

 end % END OF MOTION LOOP

 % Finalise outputs so the rest of the script knows what is going on
 if sum(abs(CurrentBotEstimate.getBotPos() - goalPos)) < 5          % if CurrentBotEstimate position is close enough to the goal, say we have arrived
     arrived = 1; 
     botPos = CurrentBotEstimate.getBotPos();
     botAng = CurrentBotEstimate.getBotAng();
 else                                                               % if we have not arrived, are off route but known (replan), or completely lost(relocalise).
     botPos = CurrentBotEstimate.getBotPos();
     botAng = CurrentBotEstimate.getBotAng();
     arrived = 0;
 end

