%% Combining PathPlanning.m and FollowPath.m - probably to be inserted into the full robot script later

%% All the setup needed for these two functions
map = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]; %default map
plotit = 1;

samples = 30;
lost = 0;
arrived = 0;
offPath = 0;

nxt = Robot();
%nxt.ang = pi/2;
%nxt.pos = [90 90];
nxt.pos = estPosition;
nxt.ang = bestAng;
%goalPos = [40 32];      % SO ITS MEASURING ITS ANGLE FROM THE Y AXIS IN A CLOCKWISE DIRECTION
goalPos = [80 80];

%% Actually using them
    %so this should plan a path, and then try to execute said path.
    
    %FollowPath has a few outcomes:
    %1. robot finds goal: arrived = 1
    %2. robot falls off path, but is caught by checkparticles, so only
    %needs to replan the route, not relocalise. arrived = 0, lost = 0
    %3. robot is lost completely, and will need to relocalise arrived = 0,
    %lost = 1 
    
    % these while loops may not be the most elegant setup.

while lost == 0 
   [pathCoord,pathLength] = PathPlanning(nxt.pos,goalPos,map,plotit);
   [pathCoord,pathLength] = pathShortening(pathCoord,pathLength,map);
    i = 1;
    nxt.pos = pathCoord(1,:);
    while arrived == 0 && lost == 0 && offPath == 0
        to = pathCoord(i+1,:);
        [nxt,arrived,lost,offPath] = pathFollow(nxt,to,goalPos);
        if i == pathLength-1
            break
        end
        i = i + 1
    end
    
    if arrived == 1
        break %and also out of the localisation loop as well
    end
end


