clear all
close all

%% Combining PathPlanning.m and FollowPath.m - probably to be inserted into the full robot script later
tic
%% All the setup needed for these two functions
% map = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]; %default map
plotit = 1;
goalPos = [88,88];

samples = 32;
lost = 0;
arrived = 0;
offPath = 0;

nxt = Robot();
map = nxt.map;
[estPosition, bestAng] = Localize(nxt, 2000, true);

nxt.pos = estPosition;
nxt.ang = bestAng;

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
    offPath = 0;
   [pathCoord,pathLength] = PathPlanning2(nxt.pos,goalPos,map,plotit);
   [pathCoord,pathLength] = pathShortening(pathCoord,pathLength,map);
   disp('Path planned and shortened')
    i = 1;
    nxt.pos = pathCoord(1,:);
    while arrived == 0 && lost == 0 && offPath == 0
        to = pathCoord(i+1,:);
        [nxt,arrived,lost,offPath] = pathFollow(nxt,to,goalPos, true);
        disp('Step performed')
        if i == pathLength-1
            break
        end
        i = i + 1;
    end
    
    if arrived == 1
        disp('NXT at goal, break out of while loop')
        break %and also out of the localisation loop as well
    end
end

toc
nxt.beep(440,200)
%mario(nxt)

nxt.close()