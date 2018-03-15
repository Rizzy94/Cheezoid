[cmPerDeg, movementNoise] = NXTCalibration(5);

COM_CloseNXT all        %prepares workspace
h = COM_OpenNXT();      %look for USB devices
COM_SetDefaultNXT(h);   %sets default handle

pause(.5)

howFar = 10;

mAB = NXTMotor('AB', 'Power', 50, 'TachoLimit', round(howFar/cmPerDeg));
mAB.SpeedRegulation = false;
mAB.SmoothStart = true;
mAB.SendToNXT();
mAB.WaitFor();
mAB.Stop('brake') 