clf('reset'); %resets figures 
clc;        %clears console
close all; %clears figures

map = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]; %default map

numScans = 4;
numScansFull = 32;
startAngle = 0;
endAngle = ((numScans-1)*2*pi)/numScans;  
angles = (startAngle:(endAngle - startAngle)/(numScans-1):endAngle);
scanLines =  [cos(angles); sin(angles)]'*100;
scanOffSet = [0, 0];
converged = 0;
dampeningFact = 0.0000000001;
redistPercentage = 0.75;
sensorNoise = 1; 
isOrthog = 0;
numParticles =2000; 

particles(numParticles,1) = BotSim; %how to set up a vector of objects
for i = 1:numParticles
    particles(i) = BotSim(map);  %each particle should use the same map as the botSim object
    particles(i).setScanConfig(scanLines,scanOffSet);
    particles(i).setSensorNoise(0);
    particles(i).setMotionNoise(0);
    particles(i).setTurningNoise(0);
    particles(i).randomPose(5); %spawn the particles in random locations
    particles(i).setBotAng(floor(rand*4)*pi/2);
end

newParticles(numParticles,1) = BotSim; 
for i = 1:numParticles
    newParticles(i) = BotSim(map);  %each particle should use the same map as the botSim object
    newParticles(i).setScanConfig(scanLines,scanOffSet);
    newParticles(i).setSensorNoise(0);
    newParticles(i).setMotionNoise(0);
    newParticles(i).setTurningNoise(0);
    newParticles(i).randomPose(5); %spawn the particles in random locations
    newParticles(i).setBotAng(floor(rand*4)*pi/2);
end

nxt = Robot(); %creates robot object
nxt.beep(440, 200); %Beep beep
n = 0;

prevPow = nxt.pUltra;
nxt.pUltra = 25;
scanA = nxt.rotScan(72);
[angleToTurn, botScan] = orthoScans(scanA);
%nxt.turn(angletoTurn);
nxt.pUltra = prevPow;
dangerZone = false;

if ((40<(botScan(1)+botScan(3)) && (botScan(1)+botScan(3))<50)&&(100<(botScan(2)+botScan(4)) && (botScan(2)+botScan(4))<110))
    if (44<botScan(2) && botScan(2)<61)&&(44<botScan(4) && botScan(4)<61)
        dangerZone = true;
    end
end

if ((40<(botScan(2)+botScan(4)) && (botScan(2)+botScan(4))<50)&&(100<(botScan(1)+botScan(3)) && (botScan(1)+botScan(3))<110))
    if (44<botScan(1) && botScan(1)<61)&&(44<botScan(3) && botScan(3)<61)
        dangerZone = true;
    end
end

[a,b] = max(botScan);
ang = mod(angleToTurn + (b-1)*pi/2, 2*pi);
if ang > pi
    ang = ang - 2*pi; 
end

if dangerZone == true
    nxt.turn(ang)
    nxt.move(max(botScan)-15)
end

% while dangerZone == true
%     canMove = false;
%     while canMove == false
%         nxt.turn(angleToTurn+pi/2);
%         dist = nxt.scan(1)
%         if dist > 50
%         	canMove = true;
%         end
%     end
%     nxt.move(dist-10)
%     prevPow = nxt.pUltra;
%     nxt.pUltra = 25;
%     scanA = nxt.rotScan(72);
%     [angleToTurn, botScan] = orthoScans(scanA);
%     %nxt.turn(angletoTurn);
%     nxt.pUltra = prevPow;
%     if (((40<(botScan(1)+botScan(3))<50)&&(100<(botScan(2)+botScan(4))<110)) || ((40<(botScan(2)+botScan(4))<50)&&(100<(botScan(1)+botScan(3))<110))) == false
%         dangerZone = false;
%     end
% end
%nxt.turn(a);

while(converged == 0)
    n=n+1;
    %scanA = nxt.rotScanStopStart(numScans); %TODO: simplify these 5 lines into 1 line
    scanA = nxt.rotScan(numScansFull);
    [angToTurn, botScan] = orthoScans(scanA);
    pause(0.3)
    %botScan = (scanA(:,1));% + scanB)/2; Not sure if this really will ever be needed now
    distances = zeros(size(scanLines,1), numParticles);

    particleProbs = zeros(numParticles,1);
    particleShift = zeros(numParticles,1);
    particlePositions = zeros(round(numParticles*redistPercentage),2);
    
    for i = 1:numParticles
        rawScan = particles(i).ultraScan(); %get a scan from the particles
        %rawScan = flipud(circshift(rawScan, -1)); %todo, more computationally effecient to flip just the bot scan not everything else.
        rawScan(rawScan > 255) = 255;
        distances(:,i) = rawScan;

        disparity = mean(abs(distances(:,i) - botScan)); 
        particleProbs(i) = exp(-(disparity^2 / (2 * sensorNoise))) + dampeningFact;
        particleShift(i) = 0;

        for shift = 1:(size(scanLines,1)-1)
            disparity = mean (abs( circshift(distances(:,i),shift) - botScan ));
            if (exp(-(disparity^2 / (2 * sensorNoise))) + dampeningFact) > particleProbs(i)
                particleProbs(i) = exp(-(disparity^2 / (2 * sensorNoise))) + dampeningFact;
                particleShift(i) = shift;
            end
        end

    end
    
    [maxProb, maxProbParticleInd] = max(particleProbs);
    
    %%
    clf
    particles(1).drawMap()
    hold on
    for i = 1:numParticles
        particles(i).drawBot(3);
    end
    drawnow;

    particleProbs = particleProbs/sum(particleProbs);
    particleDist = cumsum(particleProbs);

    for i = 1:numParticles
        particles(i).setBotAng( particles(i).getBotAng + ((endAngle-startAngle)/size(scanLines,1))*particleShift(i) )  %MINUS TO PLUS
    end

    for i = 1:round(numParticles*redistPercentage)
        keep = size(particleDist(particleDist<rand),1) + 1;
        newParticles(i).setBotPos( particles(keep).getBotPos() + (randn(2,1)/2)');
        %newParticles(i).setBotAng( mod(particles(keep).getBotAng() + pi/size(scanLines,1) , 2*pi) + randn(1)/10)
        newParticles(i).setBotAng( particles(keep).getBotAng() + randn(1)/20)
        particlePositions(i,:) = getBotPos(newParticles(i));
    end   

    %bestAng = getBotAng(newParticles(maxProbParticleInd));
    
    for i = (round(numParticles*redistPercentage)+1):numParticles
        newParticles(i).randomPose(5);
        newParticles(i).setBotAng((floor(rand*4)*pi/2))
    end

    if prod(std(particlePositions) < 8)  %from 4
        converged = 1;
    end

    for i = 1:numParticles
        particles(i).setBotAng(newParticles(i).getBotAng());
        particles(i).setBotPos(newParticles(i).getBotPos());
    end

    %%
    clf
    particles(1).drawMap()
    hold on
    for i = 1:numParticles
        particles(i).drawBot(3);
    end
    drawnow;
    
%     if true %n == 0 || botScan(1) <= 30
%         %auxVar = scanLines(botScan == max(botScan),:);
%         %turn = atan2(auxVar(1,2), auxVar(1,1));
%         turn = 0;
%     else
%         turn = 0;
%     end
    %converged = 0;  %TAKE ME OUT
    
    if converged == 0

        move = 5;   %4
        %if prod(botScan([1:round(numScans/6), (size(botScan,1)-round(numScans/6)):size(botScan,1)]) > 1.2*move) == 0
        if isOrthog == 0
           nxt.turn(angleToTurn);
           isOrthog = 1;
           scanA = nxt.rotScan(4);
        end
        if (scanA(1) < 4*move)
            foundRoute = 0;
            while foundRoute == 0
                nxt.turn(pi/2);
                %scanA = nxt.rotScanStopStart(numScans);
                scanA = nxt.rotScan(numScansFull);
                [ang, botScan] = orthoScans(scanA);
                %nxt.turn(ang);
                %botScan = (scanA(:,1));% + scanB)/2;
                %if prod(botScan([1:5, (size(botScan,1)-5):size(botScan,1)]) > 1.5*move) == 1
                if (scanA(1) > 4*move)
                    foundRoute = 1;
                end
                for i =1:numParticles %for all the particles. 
                    particles(i).turn(pi/2 + randn/20); %turn the particle in the same way as the real robot
                end
            end
        end
        nxt.move(move);
        for i =1:round(numParticles/1.25) %for half the particles. 
            particles(i).move( move + randn ); %move the particle with some noise
            if particles(i).insideMap() == 0
                particles(i).randomPose(5);
                particles(i).setBotAng((floor(rand*4)*pi/2))
            end
        end
        for i = (round(numParticles/1.25) + 1):numParticles %for the other half the particles. 
            particles(i).move( move*rand ); %move the particle less than the bot was supposed to move
            if particles(i).insideMap() == 0
                particles(i).randomPose(5);
                particles(i).setBotAng(floor(rand*4)*pi/2)
            end
        end
        clf
        particles(1).drawMap()
        hold on
        for i = 1:numParticles
            particles(i).drawBot(3);
        end
        drawnow;
    end
    
    %%
    clf
    particles(1).drawMap()
    hold on
    for i = 1:numParticles
        particles(i).drawBot(3);
    end
    drawnow;
        
end

estPosition = mean(particlePositions);

numScans = 72;
startAngle = 0;
endAngle = ((numScans-1)*2*pi)/numScans;  
angles = (startAngle:(endAngle - startAngle)/(numScans-1):endAngle);
scanLines =  [cos(angles); sin(angles)]'*100;
scanOffSet = [0, 0];
botGhost = BotSim(map);
botGhost.setScanConfig(scanLines,scanOffSet);
botGhost.setSensorNoise(0);
botGhost.setMotionNoise(0);
botGhost.setTurningNoise(0);
botGhost.setBotPos(estPosition); %spawn the particles in random locations
botGhost.setBotAng(0);
prevPow = nxt.pUltra;
nxt.pUltra = 25;
orientationScan = nxt.rotScan(numScans);
nxt.pUltra = prevPow;
ghostScan = botGhost.ultraScan();
score = zeros(numScans,1);
for i = 1:(numScans)
    auxScan = circshift( ghostScan, -i ) ;  %CHANGED i TO -i
    score(i) = sum(abs( orientationScan(orientationScan>=9) - auxScan(orientationScan>=9) ));
    %score(i) = sum(abs( orientationScan - circshift( ghostScan, i ) ));
end
[minScanVal, minScanInd] = min(score);
bestAng = ((minScanInd-1)/numScans)*(2*pi);% + pi;    %added plus pi (it's dubious, check)
botGhost.setBotAng(bestAng);
%disparity = mean (abs( circshift(distances(:,i),shift) - botScan ));
            
%estAng = bestAng;

clf
particles(1).drawMap()
hold on
for i = 1:numParticles
    particles(i).drawBot(3);
end
drawnow;

clf
botGhost.drawMap()
hold on
botGhost.drawBot(10);
drawnow;
nxt.close();

%PlanAndFollow;