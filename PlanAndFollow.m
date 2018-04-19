%% Combining PathPlanning.m and FollowPath.m - probably to be inserted into the full robot script later

%% All the setup needed for these two functions
map = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]; %default map
plotit = 1;

botPos = [22 87];
% botPos = [45 22];       % will come from localise
botAng = 3*pi/2;    % PRETTY SURE IT THINKS PI/2 IS PARALLEL WITH THE POSITIVE X AXIS. WHY? GREAT QUESTION
goalPos = [40 20];      % SO ITS MEASURING ITS ANGLE FROM THE Y AXIS IN A CLOCKWISE DIRECTION
samples = 30;
lost = 0;
arrived = 0;
nxt = Robot();

%% Actually using them
    %so this should plan a path, and then try to execute said path.
    
    %FollowPath has a few outcomes:
    %1. robot finds goal: arrived = 1
    %2. robot falls off path, but is caught by checkparticles, so only
    %needs to replan the route, not relocalise. arrived = 0, lost = 0
    %3. robot is lost completely, and will need to relocalise arrived = 0,
    %lost = 1 
    
    % these while loops may not be the most elegant setup.

%while lost == 0 
    [pathCoord,pathLength] = PathPlanning(botPos,goalPos,map,plotit);
%     [pathCoord,pathLength] = pathShortening(pathCoord,pathLength,[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]);
    
    while arrived == 0 && lost == 0 
        [botPos,botAng,arrived,lost] = FollowPath(nxt, pathCoord,botPos,botAng,pathLength,samples,map,plotit);
    end
    
    if arrived == 1
       % break
    end
%end


