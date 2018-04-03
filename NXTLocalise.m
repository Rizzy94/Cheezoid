[cmPerDeg, movementNoise, degPerDeg, turningNoise] = NXTFullCalibration(1, 500, 50, 1500, 50);
% cmPerDeg = 
% movementNoise =
% degPerDeg = 
% turningNoise = 

map = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]; %default map
target = [80,80];

startAngle =0;    
samples = 72;
endAngle = ((samples-1)*2*pi)/samples;  
angles = (startAngle:(endAngle - startAngle)/(samples-1):endAngle);
scanLines =  [cos(angles); sin(angles)]'*100;
scanOffSet = [0, 0];

dampeningFact = 0.00001;
redistPercentage = 0.95;
maxIterations = 25;
sensorNoise = 1;    %UK

robotMotorPow = 50;

numParticles = 500; 
plotMe = true;

particles(numParticles,1) = BotSim; %how to set up a vector of objects
for i = 1:numParticles
    particles(i) = BotSim(map);  %each particle should use the same map as the botSim object
    particles(i).setScanConfig(scanLines,scanOffSet);
    particles(i).setSensorNoise(0);
    particles(i).setMotionNoise(0);
    particles(i).setTurningNoise(0);
    particles(i).randomPose(0); %spawn the particles in random locations
end

newParticles(numParticles,1) = BotSim; 
for i = 1:numParticles
    newParticles(i) = BotSim(map);  %each particle should use the same map as the botSim object
    newParticles(i).setScanConfig(scanLines,scanOffSet);
    newParticles(i).setSensorNoise(0);
    newParticles(i).setMotionNoise(0);
    newParticles(i).setTurningNoise(0);
    newParticles(i).randomPose(0); %spawn the particles in random locations
end

n = 0;
converged = 0; %The filter has not converged yet
distances = zeros(size(scanLines,1), numParticles);
particleProbs = zeros(numParticles,1);
particleShift = zeros(numParticles,1);
particlePositions = zeros(round(numParticles*redistPercentage),2);



while(converged == 0 && n < maxIterations) %%particle filter loop
    
    n = n+1; %increment the current number of iterations
    botScan = NXTUltraScan(samples, round(360/degPerDeg), 'Left', robotMotorPow); %get a scan from the real robot.

    for i = 1:numParticles
        rawScan = particles(i).ultraScan(); %get a scan from the particles
        rawScan(((rawScan > 255) + (rawScan < 2)) > 0) = 255;
        distances(:,i) = rawScan; %NEW, sets scan values above 255 to 255 
        
        %NEW: after ultraScan we'll scatter the particles a little bit,
        %probs wont work
        %particles(i).setBotPos( particles(i).getBotPos() + 3*randn(1,2) )
        %particles(i).setBotAng( particles(i).getBotAng() + *randn )
        
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
    
    particleProbs = particleProbs/sum(particleProbs);
    particleDist = cumsum(particleProbs);
    
    % add ((endAngle-startAngle)/samples)*shift to each particle angle
    
    for i = 1:numParticles
        particles(i).setBotAng( mod(particles(i).getBotAng - ((endAngle-startAngle)/size(scanLines,1))*particleShift(i), 2*pi) )
    end
    
    for i = 1:round(numParticles*redistPercentage)
        keep = size(particleDist(particleDist<rand),1) + 1;
        newParticles(i).setBotPos( particles(keep).getBotPos() + randn(1,2)/1.5 );
        newParticles(i).setBotAng( mod(particles(keep).getBotAng() + pi/size(scanLines,1) + rand/10, 2*pi) )
        particlePositions(i,:) = getBotPos(newParticles(i));
    end   
    
    for i = (round(numParticles*redistPercentage)+1):numParticles
        newParticles(i).randomPose(0);
    end
    
    if prod(std(particlePositions) < 3)  %from 4
        converged = 1;
    end
    
    if plotMe == true
        clf;        %clears figures
        newParticles(1).drawMap();
        %botSim.drawBot(10);
        plot(target(1), target(2),'r*');
        for i = 1:numParticles
            newParticles(i).drawBot(3);
        end
        drawnow
    end
   
    for i = 1:numParticles
        particles(i).setBotAng(newParticles(i).getBotAng());
        particles(i).setBotPos(newParticles(i).getBotPos());
    end
    
    % here they just turn in cicles as an example
    
    %turn = 0.5;
    if n == 0 || botScan(1) <= 30
        auxVar = scanLines(botScan == max(botScan),:);
        turn = atan2(auxVar(1,2), auxVar(1,1));
    else
        turn = 0;
    end
    
    move = 10;   %4
    
    if converged == 0
        NXTTurn(turn, degPerDeg, robotMotorPow); %turn the real robot.  
        NXTMove(move, cmPerDeg, robotMotorPow); %move the real robot. 
        for i =1:numParticles %for all the particles. 
            particles(i).turn(turn); %turn the particle in the same way as the real robot
            particles(i).move(move); %move the particle in the same way as the real robot
            if particles(i).insideMap() == 0
                particles(i).randomPose(0);
            end
        end
        
    end
end

estPosition = mean(particlePositions);

