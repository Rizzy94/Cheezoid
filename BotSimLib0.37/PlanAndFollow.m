%% Combining PathPlanning.m and FollowPath.m - probably to be inserted into the full robot script later

%% All the setup needed for these two functions
map = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]; %default map
plotit = 1;

botPos = [10 10];       % will come from localise
goalPos = [80 90];


%% Actually using them
    %so this should plan a path, and then try to execute said path.
    
    %FollowPath has a few outcomes:
    %1. robot finds goal, and arrived = 1
    %2. robot falls off path, but is caught by checkparticles, so only
    %needs to replan the route, not relocalise. arrived = 0, lost = 0
    %3. robot is lost completely, and will need to relocalise arrived = 0,
    %lost = 1 
    
    % these while loops may not be the most elegant setup.

while lost == 0
    [pathCoord,pathLength] = PathPlanning(botPos,goalPos,map,plotit);
    while arrived == 0 && lost == 0 
        [botPos,botAng,arrived,lost] = FollowPath(pathCoord,botPos,botAng,pathLength,map,plotit);
    end
    
    if arrived == 1
        break
    end
end


