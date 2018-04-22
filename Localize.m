function [estPosition, estAngle, botGhost] = Localize(nxt, numParticles, plotMe)

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
sensorNoise = 1; 
isOrthog = 0;
numParticles =2000; 

end