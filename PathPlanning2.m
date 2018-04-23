function [path,pathLength] = PathPlanning2(botPos,goalPos,map,plotit)

botSim = BotSim(map);
botSim.setBotPos(botPos);
target = goalPos;
doPlot = plotit;
estPosition = botPos;
numScans = 100;
startAngle = 0;
endAngle = ((numScans-1)*2*pi)/numScans;  
angles = (startAngle:(endAngle - startAngle)/(numScans-1):endAngle);
scanLines =  [cos(angles); sin(angles)]'*100;
scanOffSet = [0, 0];

limsMin = min(map); % minimum limits of the map
limsMax = max(map); % maximum limits of the map
dims = limsMax-limsMin; %dimension of the map
res = 3; %sampling resouloution in cm
safety = 20;
iterators = dims/res;
iterators = ceil(iterators)+[1 1]; %to counteract 1 based indexing
hold on
%loops through the grid indexes and tests if they are inside the map
discreteMapPoints = zeros(2, iterators(1)*iterators(2) + 2);
pointsInDiscreteMap = 0;

for i = 1:iterators(2)
    for j = 1:iterators(1)
        testPos = limsMin + [j-1 i-1]*res; %to counteract 1 based indexing
        %notice, that i and j have also been swapped here so that the only
        %thing left to do is invert the y axis. 
        if botSim.pointInsideMap(testPos)
            proximityTester = BotSim(map);
            proximityTester.setScanConfig(scanLines,scanOffSet);
            proximityTester.setSensorNoise(0);
            proximityTester.setMotionNoise(0);
            proximityTester.setTurningNoise(0);
            proximityTester.setBotPos(testPos);
            proximity = proximityTester.ultraScan();
            if min(proximity) > safety
                pointsInDiscreteMap = pointsInDiscreteMap + 1;
                discreteMapPoints(:, pointsInDiscreteMap) = testPos;
            end
        end
    end
end

if pointsInDiscreteMap > 0
    discreteMapPoints = discreteMapPoints(:,1:(pointsInDiscreteMap+2));
    discreteMapPoints(:, pointsInDiscreteMap+1) = target;
    discreteMapPoints(:, pointsInDiscreteMap+2) = botPos;
    pointsInDiscreteMap = pointsInDiscreteMap + 2;
end

if doPlot == 1
    clf
    botSim.drawMap();
    botSim.drawBot(10);
    plot(target(1), target(2),'r*');
    for i = 1:size(discreteMapPoints,2)
        plot(discreteMapPoints(1,i), discreteMapPoints(2,i) , 'o');
    end
    drawnow;
end

%% discreteMapPoints has via points, we now need a martix which tells us which direction each via point can lead to

minDist = distance(estPosition , discreteMapPoints(:,1)');
closestVertex = 1;
for i = 2:(size(discreteMapPoints(1,:), 2) - 2)
    if distance(estPosition , discreteMapPoints(:,i)') < minDist
        minDist = distance(estPosition , discreteMapPoints(:,i)');
        closestVertex = i;
    end
end

minDist = distance(target , discreteMapPoints(:,1)');
closestVertexToTarget = 1;
for i = 2:(size(discreteMapPoints(1,:), 2) - 2)
    if distance(target , discreteMapPoints(:,i)') < minDist
        minDist = distance(target , discreteMapPoints(:,i)');
        closestVertexToTarget = i;
    end
end


possibleDirections = zeros(5, (size(discreteMapPoints(1,:), 2) - 2));  %up right down left target

for i = 1:(size(discreteMapPoints(1,:), 2) - 2)
    if sum(sum((discreteMapPoints(:,i) + [0, res]') == discreteMapPoints(:,:), 1) == 2) == 1    %up
        possibleDirections(1,i) = 1;
    end
    if sum(sum((discreteMapPoints(:,i) + [res, 0]') == discreteMapPoints(:,:), 1) == 2) == 1    % right
        possibleDirections(2,i) = 1;
    end
    if sum(sum((discreteMapPoints(:,i) + [0, -res]') == discreteMapPoints(:,:), 1) == 2) == 1    % down
        possibleDirections(3,i) = 1;
    end
    if sum(sum((discreteMapPoints(:,i) + [-res, 0]') == discreteMapPoints(:,:), 1) == 2) == 1    %left
        possibleDirections(4,i) = 1;
    end
    if distance( discreteMapPoints(:,i)' , target) <= res
        possibleDirections(5,i) = 1;
    end
end

startPoint = discreteMapPoints(:,closestVertex);
open = zeros(2, size(discreteMapPoints,2));
numOpen = 1;

priority = zeros(1, size(discreteMapPoints,2));
cameFrom = zeros(2, size(discreteMapPoints,2));

hasBeenOpen = zeros(1, size(discreteMapPoints,2));

open(:,1) = startPoint;
cameFrom(:,closestVertex) = estPosition;
cameFrom(:,closestVertexToTarget) = target;


costVect = repmat(size(discreteMapPoints,2),size(discreteMapPoints,2),1);
costVect(sum(discreteMapPoints == startPoint,1) == 2) = 0;
succesorNodes = zeros(2,5);

while numOpen > 0

    for i = 1:numOpen
        priority(i) = distance( open(:,i)' , target );
    end

    bestPriorityNodes = open( :, priority(1:numOpen) == min(priority(1:numOpen)) );
    nodeCurrent = bestPriorityNodes(:,1);
    currentNodeIndex = sum(discreteMapPoints == nodeCurrent,1) == 2 ;
    currentNodeIndex = ((cumsum(currentNodeIndex) == max(cumsum(currentNodeIndex))) .* currentNodeIndex) == 1;
    hasBeenOpen( currentNodeIndex ) = 1;

    if sum(nodeCurrent' == target) == 2
        %We're there
        break
        %return
    end

    numSuccesors = sum(possibleDirections( :,currentNodeIndex ));
    succesor = 1;

    if possibleDirections(1,currentNodeIndex ) % Up
        succesorNodes(:,succesor) = nodeCurrent + [0, res]';
        succesor = succesor + 1;
    end
    if possibleDirections(2,currentNodeIndex ) % Right
        succesorNodes(:,succesor) = nodeCurrent + [res, 0]';
        succesor = succesor + 1;
    end
    if possibleDirections(3,currentNodeIndex ) % Down
        succesorNodes(:,succesor) = nodeCurrent + [0, -res]';
        succesor = succesor + 1;
    end
    if possibleDirections(4,currentNodeIndex ) % Left
        succesorNodes(:,succesor) = nodeCurrent + [-res, 0]';
        succesor = succesor + 1;
    end
    if possibleDirections(5,currentNodeIndex ) % Target
        succesorNodes(:,succesor) = target';
    end

    for i = 1:numSuccesors

        succesorNodeIndex = sum(discreteMapPoints == succesorNodes(:,i),1) == 2 ;
        succesorNodeIndex = ((cumsum(succesorNodeIndex) == max(cumsum(succesorNodeIndex))) .* succesorNodeIndex)==1;
        succesorCurrentCost = costVect(currentNodeIndex) + 1;

        if succesorCurrentCost < costVect(succesorNodeIndex)
            costVect(succesorNodeIndex) = succesorCurrentCost;
            cameFrom(:,succesorNodeIndex) = discreteMapPoints(:,currentNodeIndex);
        end

        if hasBeenOpen(succesorNodeIndex) == 0
            numOpen = numOpen + 1;
            open(:,numOpen) = discreteMapPoints(:,succesorNodeIndex);
        end
    end


    % We want to swap the current node in open to the node in numOpens
    % position

    open(:,[numOpen, find(sum(open == nodeCurrent,1) == 2)]) = open(:,[find(sum(open == nodeCurrent,1) == 2), numOpen]);
    open(:,numOpen) = [0;0];
    numOpen = numOpen - 1;

end

path = zeros(2, size(discreteMapPoints,2) + 2);
path(:, 1) = target;
lenPath = 1;
home = false;
currentStep = discreteMapPoints(:,closestVertexToTarget);

while home == false
    lenPath = lenPath + 1;
    auxIndex = sum(discreteMapPoints == currentStep,1) == 2;
    auxIndex = ((cumsum(auxIndex) == max(cumsum(auxIndex))) .* auxIndex) == 1;

    from = cameFrom(:, auxIndex);
    path(:, lenPath) = from(:,1);
    currentStep = cameFrom(:, auxIndex );
    if prod(currentStep == estPosition') == 1
        home = true;
    end
end

path = flip(path(:, 1:lenPath),2);
path = path';
pathLength = size(path, 1) - 1;

end
