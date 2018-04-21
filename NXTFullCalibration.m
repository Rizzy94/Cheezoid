function [cmPerDeg, movementNoise, degPerDeg, turningNoise] = NXTFullCalibration(numTrials, wheelDegsMove, ...
    wheelPowMove, wheelDegsTurn, wheelPowTurn)

% COM_CloseNXT all        %prepares workspace
% h = COM_OpenNXT();      %look for USB devices
% COM_SetDefaultNXT(h);   %sets default handle

%% Movement Calibration

ready = 'NoGo';

while strcmp(ready,'GO') == 0
    ready = input(['Type GO when you are ready to measure robot movement' newline], 's');
end

pause(.5)

mAB = NXTMotor('AB', 'Power', 
wheelPowMove, 'TachoLimit', wheelDegsMove);
mAB.SpeedRegulation = false;
mAB.SmoothStart = true;

dists = zeros(numTrials, 1);

for i = 1:numTrials
    
    mAB.SendToNXT();
    mAB.WaitFor();
    mAB.Stop('brake') 
    dists(i) = input(['How many cm did the robot move?' newline]);

end

cmPerDeg = mean(dists) / wheelDegsMove;
movementNoise = std(dists);

sprintf('This robot moves %d cm per 1 degree of wheel rotation.' , cmPerDeg)
disp(newline)
sprintf('This robot has approximately %d movementNoise.' , movementNoise)
disp(newline)

%% Angular Calibration

ready = 'NoGo';

while strcmp(ready,'GO') == 0
    ready = input(['Type GO when you are ready to measure robot movement' newline], 's');
end

mA = NXTMotor('A', 'Power', wheelPowTurn, 'TachoLimit', wheelDegsTurn);
mB = NXTMotor('B', 'Power', -wheelPowTurn, 'TachoLimit', wheelDegsTurn);
mA.SpeedRegulation = false;
mB.SpeedRegulation = false;
mA.SmoothStart = true;
mB.SmoothStart = true;

angs = zeros(numTrials, 1);

for i = 1:numTrials
    
    mA.SendToNXT();
    mB.SendToNXT();
    
    mA.WaitFor();
    mB.WaitFor();
    
    mA.Stop('brake') 
    mB.Stop('brake')

    angs(i) = input(['How much did it turn in degrees?' newline]);

end

degPerDeg = mean(angs) / wheelDegsTurn;
turningNoise = std(angs);

sprintf('This robot turns %d degs per 1 degree of wheel rotations.' , degPerDeg)
disp(newline)
sprintf('This robot has approximately %d turningNoise.' , turningNoise)
disp(newline)



end

