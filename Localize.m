function [estPosition, bestAng] = Localize(nxt, numParticles, plotMe)

map=nxt.map;

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
sensorNoise = .4; 
isOrthog = 0;

particles(numParticles,1) = BotSim;
for i = 1:numParticles
    particles(i) = BotSim(map); 
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



prevPow = nxt.pUltra;
nxt.pUltra = 25;
scanA = nxt.rotScan(72);
[angleToTurn, botScan] = orthoScans(scanA);
%nxt.turn(angletoTurn);
while (sum(botScan < 9) > 0)
    scanA = nxt.rotScan(72);
    [angleToTurn, botScan] = orthoScans(scanA);
end
dangerZone = false;
if ((40<(botScan(1)+botScan(3)) && (botScan(1)+botScan(3))<50)&&(100<(botScan(2)+botScan(4)) && (botScan(2)+botScan(4))<110))
    if (44<botScan(2) && botScan(2)<61)&&(44<botScan(4) && botScan(4)<61)
        dangerZone = true;
    end
end
nxt.pUltra = prevPow;

if ((40<(botScan(2)+botScan(4)) && (botScan(2)+botScan(4))<50)&&(100<(botScan(1)+botScan(3)) && (botScan(1)+botScan(3))<110))
    if (44<botScan(1) && botScan(1)<61)&&(44<botScan(3) && botScan(3)<61)
        dangerZone = true;
    end
end

[~,b] = max(botScan);
ang = mod(angleToTurn + (b-1)*pi/2, 2*pi);
if ang > pi
    ang = ang - 2*pi; 
end

if dangerZone == true
    nxt.turn(ang)
    nxt.move(max(botScan)-15)
end


while(converged == 0)
    scanA = nxt.rotScan(numScansFull);
    [~, botScan] = orthoScans(scanA);
    %pause(0.3)
    distances = zeros(size(scanLines,1), numParticles);
    particleProbs = zeros(numParticles,1);
    particleShift = zeros(numParticles,1);
    particlePositions = zeros(round(numParticles*redistPercentage),2);
    
    for i = 1:numParticles
        rawScan = particles(i).ultraScan(); 
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
    
    %%
    if plotMe == true
        clf
        particles(1).drawMap()
        hold on
        for i = 1:numParticles
            particles(i).drawBot(3);
        end
        drawnow;
    end

    particleProbs = particleProbs/sum(particleProbs);
    particleDist = cumsum(particleProbs);

    for i = 1:numParticles
        particles(i).setBotAng( particles(i).getBotAng + ((endAngle-startAngle)/size(scanLines,1))*particleShift(i) )  %MINUS TO PLUS
    end

    for i = 1:round(numParticles*redistPercentage)
        keep = size(particleDist(particleDist<rand),1) + 1;
        newParticles(i).setBotPos( particles(keep).getBotPos() + (randn(2,1)/2)');
        newParticles(i).setBotAng( particles(keep).getBotAng() + randn(1)/20)
        particlePositions(i,:) = getBotPos(newParticles(i));
    end   
    
    for i = (round(numParticles*redistPercentage)+1):numParticles
        newParticles(i).randomPose(5);
        newParticles(i).setBotAng((floor(rand*4)*pi/2))
    end

    if prod(std(particlePositions) < 8)  
        converged = 1;
    end

    for i = 1:numParticles
        particles(i).setBotAng(newParticles(i).getBotAng());
        particles(i).setBotPos(newParticles(i).getBotPos());
    end

    %%
    if plotMe == true
        clf
        particles(1).drawMap()
        hold on
        for i = 1:numParticles
            particles(i).drawBot(3);
        end
        drawnow;
    end
    
    if converged == 0

        move = 5;  
        if isOrthog == 0
           nxt.turn(angleToTurn);
           isOrthog = 1;
           scanA = nxt.rotScan(4);
        end
        if (scanA(1) < 4*move)
            foundRoute = 0;
            while foundRoute == 0
                nxt.turn(pi/2);
                scanA = nxt.rotScan(numScansFull);
                if (scanA(1) > 4*move)
                    foundRoute = 1;
                end
                for i =1:numParticles %for all the particles. 
                    particles(i).turn(pi/2 + randn/20); %turn the particle in the same way as the real robot
                end
            end
        end
        nxt.move(move);
        for i =1:round(numParticles/1.25) 
            particles(i).move( move + randn ); 
            if particles(i).insideMap() == 0
                particles(i).randomPose(5);
                particles(i).setBotAng((floor(rand*4)*pi/2))
            end
        end
        for i = (round(numParticles/1.25) + 1):numParticles 
            particles(i).move( move*rand ); %move the particle less than the bot was supposed to move
            if particles(i).insideMap() == 0
                particles(i).randomPose(5);
                particles(i).setBotAng(floor(rand*4)*pi/2)
            end
        end
        
        if plotMe == true
            clf
            particles(1).drawMap()
            hold on
            for i = 1:numParticles
                particles(i).drawBot(3);
            end
            drawnow;
        end
    end
    
    %%
    if plotMe == true
        clf
        particles(1).drawMap()
        hold on
        for i = 1:numParticles
            particles(i).drawBot(3);
        end
        drawnow;
    end
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
    auxScan = circshift( ghostScan, -i ) ; 
    score(i) = sum(abs( orientationScan(orientationScan>=9) - auxScan(orientationScan>=9) ));
end
[~, minScanInd] = min(score);
bestAng = ((minScanInd-1)/numScans)*(2*pi);
botGhost.setBotAng(bestAng);

if plotMe == true
    clf
    particles(1).drawMap()
    hold on
    for i = 1:numParticles
        particles(i).drawBot(3);
    end
    drawnow;
end

if plotMe == true
    clf
    botGhost.drawMap()
    hold on
    botGhost.drawBot(10);
    drawnow;
end

end