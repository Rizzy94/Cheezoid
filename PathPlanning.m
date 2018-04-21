function [pathCoord,pathLength] = PathPlanning(botPos,goalPos,map,plotit)

plotBot = BotSim(map);
%plotBot.setMap(map);
plotBot.setBotPos(botPos);
g = BotSim(map);
g.setBotPos(goalPos);

v = 0;
while v == 0    % path planning
    %% IF THERE'S A STRAIGHT PATH FROM BOT TO GOAL, GO STRAIGHT THERE
    startPos = botPos; % used to be a getBotPos so i think [x y]

    tester = BotSim(map);
    tester.setMap(map);
    leaves = 0;
    dist = ceil(sqrt((startPos(1)-goalPos(1))^2 + (startPos(2)-goalPos(2))^2));
    if dist < 20
        interpolPointsX = linspace(startPos(1),goalPos(1),dist)';
        interpolPointsY = linspace(startPos(2),goalPos(2),dist)';
        interpolPoints = [interpolPointsX, interpolPointsY];

        if sum(tester.pointInsideMap(interpolPoints) == 0)
            leaves = leaves + 1;
        end

        if leaves == 0
            pathCoord = [flip(startPos);flip(goalPos)];
            %plot(goalPos(1),goalPos(2),'o','MarkerEdgeColor',[1 0 0])
            break    
        end
    end
    %% Working out node stuff
    limsMin = min(map); % minimum limits of the map
    limsMax = max(map); % maximum limits of the map
    dims = limsMax-limsMin; %dimension of the map
    res = 10; %sampling resolution in cm
    iterators = dims/res;
    iterators = ceil(iterators)+[1 1]; %[1 1] to counteract 1 based indexing. This line tells us how many points in each direction
    mapArray = zeros(iterators(2),iterators(1)); %preallocate for speed

    %% Loop through the grid indexes and test if they are inside the map (Can get rid of the plotting for speed)

    for i = 1:iterators(2) % i is over y
        for j = 1:iterators(1) % j is over x
            testPos = limsMin + [j-1 i-1]*res; %to counteract 1 based indexing --- THIS SHIFTS FROM NODE COORDINATES TO REAL ONES

            mapArray(i,j) = tester.pointInsideMap(testPos);
            if plotit == 1
                if mapArray(i,j) == 1
                    plot(testPos(1),testPos(2),'*');%inside map
                else
                    plot(testPos(1),testPos(2),'o');%outside map
                end
            end
        end
    end

    %% Shuffle the map down one and across one so the map is surrounded by zeros. Makes sure algorithm works - shuffle nodes back by [1 1] to fix at end
    siz = size(mapArray);
    zerosRow1 = zeros(1,siz(2));          % iterators(2) is x, iterators(1) is y
    zerosCol1 = zeros(siz(1)+1,1);
    zerosRow2 = zeros(1,siz(2)+1);
    zerosCol2 = zeros(siz(1)+2,1);
    mapArray2 = [zerosRow1;mapArray];             % mapArray2 is (iterators(1)+2 x iterators(2)+2)
    mapArray2 = [zerosCol1,mapArray2];
    mapArray2 = [mapArray2;zerosRow2];
    mapArray2 = [mapArray2,zerosCol2];
    sizMap2 = size(mapArray2);

    %% Shrink the map to avoid leaving it
    mapShrunk = zeros(sizMap2(1),sizMap2(2)); % Same size as mapArray2

    nb = 0;      % Number of neighbours to a point that are within the map
    %for g = 1:2
    for i = 2:sizMap2(2)-1 %over x. leave the edges so neighbours check works. (edges are black anyway)
        for j = 2:sizMap2(1)-1  %over y
            if mapArray2(j,i) == 1
                nb = 0;
               for k = 1:3
                   for m = 1:3
                       nb = nb + mapArray2(j-2+m,i-2+k);
                   end
               end
               if sum(nb) == 9          % If a point is at an edge, it is discarded

                   mapShrunk(j-1,i-1) = 1;  %SO I THINK THIS SHUFFLES MAPSHRUNK BACK BY -1, NEGATING THE SHIFT FOR MAPARRAY2
                   if plotit == 1
                        tester.drawMap();
                        plot(res*(i-2)+limsMin(1),res*(j-2)+limsMin(2),'o','MarkerEdgeColor',[0,0,0]) % -1 for starting at 1, -1 for the shift above
                   end
               end
            end
        end
    end

    if plotit == 1
        pause(1);
    end
    %% Calculate start node and end node from startPos and goalPos. --- will need to find closest node to each
    mapArrayCoord = zeros(sizMap2(1),sizMap2(2),2);
    mapArrayStartDiff = zeros(sizMap2(1),sizMap2(2));
    mapArrayGoalDiff = zeros(sizMap2(1),sizMap2(2));
    limsMinFlip = flip(limsMin);

    for j = 1:iterators(2)+2 % y 
        for i = 1:iterators(1)+2 % x
            if mapShrunk(j,i) == 1
                mapArrayCoord(j,i,:) = limsMinFlip + [j-1 i-1]*res; % Need to counterract starting at 1, and also the shift across by adding the extra line (BUT MAYBE NOT, CHANGED FROM -2)
                mapArrayStartDiff(j,i) = sum(abs(mapArrayCoord(j,i,2)-startPos(1)) + abs(mapArrayCoord(j,i,1)-startPos(2))) + 0.01; % 0.1 fixes if the bot is actually on a node (unlikely irl but ykno)
                mapArrayGoalDiff(j,i) = sum(abs(mapArrayCoord(j,i,2)-goalPos(1)) + abs(mapArrayCoord(j,i,1)-goalPos(2))) + 0.01; 
            end
        end
    end

    closestToStart = min(mapArrayStartDiff(mapArrayStartDiff > 0));
    [yStartNode,xStartNode] = find(mapArrayStartDiff == closestToStart);        % remember this can be more than one --- see plot -- pick arbitrarily
    nodeStart = [yStartNode,xStartNode];
    if plotit == 1
       plot(res*(nodeStart(2)-1)+limsMin(1),res*(nodeStart(1)-1)+limsMin(2),'o','MarkerEdgeColor','g')  % MIGHT BE WRONG WITH THE LIMSMIN
    end

    closestToGoal = min(mapArrayGoalDiff(mapArrayGoalDiff > 0));
    [yGoalNode,xGoalNode] = find(mapArrayGoalDiff == closestToGoal);         
    nodeGoal = [yGoalNode,xGoalNode];
    if plotit == 1
        plot(goalPos(1),goalPos(2),'o','MarkerEdgeColor','r');
        plot(res*(nodeGoal(2)-1)+limsMin(2),res*(nodeGoal(1)-1)+limsMin(1),'o','MarkerEdgeColor',[0 1 1])
    end
    %% Create all necessary arrays
    openScoreArray = zeros(sizMap2);
    closedArray = zeros(sizMap2);
    heuristicArray = zeros(sizMap2);
    pathLengthArray = zeros(sizMap2);
    cameFromArray = zeros(sizMap2(1),sizMap2(2),2);

    for i = 1:iterators(2)+2 % y
        for j = 1:iterators(1)+2 % x
            heuristicArray(i,j) = abs(nodeGoal(1) - i) + abs(nodeGoal(2) - j); % Fill out the heuristic array
        end
    end

    openScoreArray(nodeStart(1),nodeStart(2)) = heuristicArray(nodeStart(1),nodeStart(2)); % give the startNode a score to begin
    expandingPathArray = zeros(3,3);
    expandingScoreArray = zeros(3,3);

    %% Actual A* algorithm
    z = 0;
    u = 1;
    while z == 0   % start of a*
        if sum(openScoreArray(openScoreArray > 0)) > 0
            currentMinScore = min(openScoreArray(openScoreArray > 0));
        else
             u = 0;
             break
        end

         [yMin,xMin] = find(openScoreArray == currentMinScore);
        %     Now we need to adjust the score for each successor node attached to
        %     the current node
         for m = [0,1,2]
            for n = [0,1,2]     
                if mapShrunk(yMin(1)-1+m,xMin(1)-1+n) == 1 
                    expandingPathArray(1+m,1+n) = pathLengthArray(yMin(1),xMin(1)) + sqrt((m-1)^2+(n-1)^2);
                    expandingScoreArray(1+m,1+n) = expandingPathArray(1+m,1+n) + heuristicArray(yMin(1)-1+m,xMin(1)-1+n); 

                    if openScoreArray(yMin(1)-1+m,xMin(1)-1+n) == 0 %if the successor has not been explored, give it a score and path length
                        openScoreArray(yMin(1)-1+m,xMin(1)-1+n) = expandingScoreArray(1+m,1+n);
                        pathLengthArray(yMin(1)-1+m,xMin(1)-1+n) = expandingPathArray(1+m,1+n);
                        cameFromArray(yMin(1)-1+m,xMin(1)-1+n,:) = [yMin(1),xMin(1)];
                    elseif openScoreArray(yMin(1)-1+m,xMin(1)-1+n) > 0 && expandingPathArray(1+m,1+n) <  pathLengthArray(yMin(1)-1+m,xMin(1)-1+n) % if the successor has a score and is open and the new path is shorter, give it a new value
                        openScoreArray(yMin(1)-1+m,xMin(1)-1+n) = expandingScoreArray(1+m,1+n);
                        pathLengthArray(yMin(1)-1+m,xMin(1)-1+n) = expandingPathArray(1+m,1+n);
                        cameFromArray(yMin(1)-1+m,xMin(1)-1+n,:) = [yMin(1),xMin(1)];
                    elseif openScoreArray(yMin(1)-1+m,xMin(1)-1+n) == -1 && expandingPathArray(1+m,1+n) <  pathLengthArray(yMin(1)-1+m,xMin(1)-1+n)
                        openScoreArray(yMin(1)-1+m,xMin(1)-1+n) = expandingScoreArray(1+m,1+n);
                        pathLengthArray(yMin(1)-1+m,xMin(1)-1+n) = expandingPathArray(1+m,1+n);
                        cameFromArray(yMin(1)-1+m,xMin(1)-1+n,:) = [yMin(1),xMin(1)];
                        closedArray(yMin(1)-1+m,xMin(1)-1+n) = 0;
                    end
                end
            end
        end
        openScoreArray(yMin(1),xMin(1)) = -1;
        closedArray(yMin(1),xMin(1)) = 1;
        if closedArray(nodeGoal(1),nodeGoal(2)) == 1
            disp('Goal node found and expanded')
            goalFound = 1;
            break
        end
        if sum(openScoreArray) == -sum(mapShrunk)
            disp('Goal node not found')
            z = 1;
            break
        end
    end

    if u == 0
        break
    end
    %% Find the path from start to finish
    %  Now we need to reverse engineer the path from start to goal
    % cameFromArray(:,:,1) is y, cameFromArray(:,:,2) is x
    path = [nodeGoal(1),nodeGoal(2)]; %[y,x]
    x = 0;
    y = 0;
    c = 1;
    if goalFound == 1
       while y ~= nodeStart(1) || x ~= nodeStart(2) % NODE START IS [Y,X]      
           y = cameFromArray(path(c,1),path(c,2),1);
           x = cameFromArray(path(c,1),path(c,2),2);
           path = [path; [y,x]];
           c = c + 1;
       end
    end

    %% Plot path
    path = flip(path);
    pathCoord = res*(path-1) + limsMinFlip;
    pathCoord = [flip(startPos);pathCoord;flip(goalPos)];
    break
end
    % A* and simple path planning converge here
[pathLength,~] = size(pathCoord);
if plotit == 1
    figure(1)
    hold on
    plotBot.drawMap();
    plotBot.drawBot(10,'r');
    g.drawBot(5,'b')
    plot(pathCoord(:,2),pathCoord(:,1),'-');
    drawnow();
    pause(1);
end

pathCoord = flip(pathCoord,2);
