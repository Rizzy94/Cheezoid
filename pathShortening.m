function [shortPath,shortPathLength] = pathShortening(pathCoord,pathLength,map)

maxStep = 100;           % can't be less than 14.1 .... for res = 10
wallDist = 10;
orientAngles = 30;
numChecks = 100;
checkBot = BotSim(map);
checkBot.setBotPos(pathCoord(1,:));
checkBot.setScanConfig(checkBot.generateScanConfig(orientAngles));

shortPath = pathCoord
shortPathLength = size(shortPath,1);
pathLength = pathLength(1);

for i = 1:pathLength-1
    i
  
   
   checkBot.setBotPos(shortPath(i,:))
   
   viable = ones(size(shortPath,1),1);
   viable(1:i,1) = 0;         % set all previous points to 0. makes life easier. 0 IMPLIES POINT IS VIABLE. WEIRD I KNOW BUT IT COMES FROM HOW I SEARCH
   
   
   for j = i+1:size(shortPath,1)
       %interpolate between current position and next node in the sequence
       lineX = linspace(shortPath(i,1),shortPath(j,1),numChecks);
       lineY = linspace(shortPath(i,2),shortPath(j,2),numChecks);

       scans = zeros(orientAngles,numChecks);
       dist = sqrt((shortPath(i,1)-shortPath(j,1))^2 + (shortPath(i,2)-shortPath(j,2))^2);
              
       if dist < maxStep
           for k = 1:numChecks
               checkBot.setBotPos([lineX(1,k), lineY(1,k)]);
               scans(:,k) = checkBot.ultraScan();
           end
             
           if min(scans(:)) >= wallDist          
               viable(j,1) = 0;     
           end  
       end
   end
   % viable vector complete for going from i'th node, find furthest
   % possible
   viable
   I = find(viable,1)      % this should find the first NOT VIABLE node, so last viable node is the one before FIND FINDS THE FIRST NON ZERO ELEMENT
   
   if ~isempty(I)
       for j = min(size(shortPath,1)-1,I(1)-2):-1:i+1       % -1 would go to the last good node, which we want to keep. so then -2
           shortPath(j,:) = [];  
           removed = j
           shortPath
       end
   else
       for j = size(shortPath,1)-1:-1:i+1          
           shortPath(j,:) = [];
           removed = j
           shortPath
       end
       break
   end
   
end

shortPath
figure(2)
checkBot.drawMap();
checkBot.drawBot(3);
scatter(shortPath(:,1),shortPath(:,2))







end