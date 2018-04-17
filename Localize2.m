clf('reset'); %resets figures 
clc;        %clears console
close all; %clears figures

map = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]; %default map

numScans = 10;
startAngle = 0;
endAngle = ((numScans-1)*2*pi)/numScans;  
angles = (startAngle:(endAngle - startAngle)/(numScans-1):endAngle);
scanLines =  [cos(angles); sin(angles)]'*100;
scanOffSet = [0, 0];
converged = 0;
dampeningFact = 0.000000001;
redistPercentage = 0.95;
sensorNoise = 1; 

numParticles = 2000; 

particles(numParticles,1) = BotSim; %how to set up a vector of objects
for i = 1:numParticles
    particles(i) = BotSim(map);  %each particle should use the same map as the botSim object
    particles(i).setScanConfig(scanLines,scanOffSet);
    particles(i).setSensorNoise(0);
    particles(i).setMotionNoise(0);
    particles(i).setTurningNoise(0);
    particles(i).randomPose(10); %spawn the particles in random locations
end

newParticles(numParticles,1) = BotSim; 
for i = 1:numParticles
    newParticles(i) = BotSim(map);  %each particle should use the same map as the botSim object
    newParticles(i).setScanConfig(scanLines,scanOffSet);
    newParticles(i).setSensorNoise(0);
    newParticles(i).setMotionNoise(0);
    newParticles(i).setTurningNoise(0);
    newParticles(i).randomPose(10); %spawn the particles in random locations
end

nxt = Robot(); %creates robot object
nxt.beep(440, 200); %Beep beep
n = 0;

while(converged == 0)
    n=n+1;
    scanA = nxt.rotScan(numScans); %TODO: simplify these 5 lines into 1 line
    pause(0.3)
    scanB = nxt.rotScan(numScans);  
    pause(0.3)
    botScan = (scanA(:,1));% + scanB)/2; Not sure if this really will ever be needed now

    distances = zeros(size(scanLines,1), numParticles);

    particleProbs = zeros(numParticles,1);
    particleShift = zeros(numParticles,1);
    particlePositions = zeros(round(numParticles*redistPercentage),2);

    for i = 1:numParticles
        rawScan = particles(i).ultraScan(); %get a scan from the particles
        rawScan = flipud(circshift(rawScan, -1)); %todo, more computationally effecient to flip just the bot scan not everything else.
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
        particles(i).setBotAng( mod(particles(i).getBotAng + ((endAngle-startAngle)/size(scanLines,1))*particleShift(i), 2*pi) )  %MINUS TO PLUS
    end

    for i = 1:round(numParticles*redistPercentage)
        keep = size(particleDist(particleDist<rand),1) + 1;
        newParticles(i).setBotPos( particles(keep).getBotPos() + (randn(2,1)/2)');
        newParticles(i).setBotAng( mod(particles(keep).getBotAng() + pi/size(scanLines,1) , 2*pi) + randn(1)/10)
        particlePositions(i,:) = getBotPos(newParticles(i));
    end   

    for i = (round(numParticles*redistPercentage)+1):numParticles
        newParticles(i).randomPose(10);
    end

    %%
    clf
    particles(1).drawMap()
    hold on
    for i = 1:numParticles
        particles(i).drawBot(3);
    end
    drawnow;

    if prod(std(particlePositions) < 5)  %from 4
        converged = 1;
    end

    for i = 1:numParticles
        particles(i).setBotAng(newParticles(i).getBotAng());
        particles(i).setBotPos(newParticles(i).getBotPos());
    end


%     if true %n == 0 || botScan(1) <= 30
%         %auxVar = scanLines(botScan == max(botScan),:);
%         %turn = atan2(auxVar(1,2), auxVar(1,1));
%         turn = 0;
%     else
%         turn = 0;
%     end

    if converged == 0

        move = 10;   %4
        if prod(botScan([1:5, (size(botScan,1)-5):size(botScan,1)]) > 2*move) == 0
            foundRoute = 0;
            while foundRoute == 0
                nxt.turn(pi/2);
                scanA = nxt.rotScan(numScans);
                pause(0.3)
                scanB = nxt.rotScan(numScans);  
                pause(0.3)
                botScan = (scanA(:,1));% + scanB)/2;
                if prod(botScan([1:5, (size(botScan,1)-5):size(botScan,1)]) > 2*move) == 1
                    foundRoute = 1;
                end
                for i =1:numParticles %for all the particles. 
                    particles(i).turn(pi/2 + randn/5); %turn the particle in the same way as the real robot
                end
            end
        end
        nxt.move(move);
        for i =1:numParticles %for all the particles. 
            particles(i).move( move + randn ); %turn the particle in the same way as the real robot
            if particles(i).insideMap() == 0
                particles(i).randomPose(10);
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
        
end

estPosition = mean(particlePositions);

clf
particles(1).drawMap()
hold on
for i = 1:numParticles
    particles(i).drawBot(3);
end
drawnow;
nxt.close();
