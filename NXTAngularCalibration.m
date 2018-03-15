function [degPerDeg, turningNoise] = NXTAngularCalibration(numTrials)

ready = 'NoGo';

wheelDegs = 100;
wheelPow = 50;

while strcmp(ready,'GO') == 0
    ready = input(['Type GO when you are ready to measure robot movement' newline], 's');
end

COM_CloseNXT all        %prepares workspace
h = COM_OpenNXT();      %look for USB devices
COM_SetDefaultNXT(h);   %sets default handle

pause(.5)

mA = NXTMotor('A', 'Power', wheelPow, 'TachoLimit', wheelDegs);
mB = NXTMotor('B', 'Power', -wheelPow, 'TachoLimit', wheelDegs);
mA.SpeedRegulation = false;
mB.SpeedRegulation = false;
mA.SmoothStart = true;
mB.SmoothStart = true;

dists = zeros(numTrials, 1);

for i = 1:numTrials
    
    mA.SendToNXT();
    mB.SendToNXT();
    
    mA.WaitFor();
    mB.WaitFor();
    
    mA.Stop('brake') 
    mB.Stop('brake')

    angs(i) = input(['How much did it turn in degrees?' newline]);

end

degPerDeg = mean(angs) / wheelDegs;
turningNoise = std(angs);

sprintf('This robot turns %d degs per 1 degree of wheel rotations.' , degPerDeg)
disp(newline)
sprintf('This robot has approximately %d turningNoise.' , turningNoise)
disp(newline)

end