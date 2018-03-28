%% This function will run the calibration, then do a move, a turn, then a scan

% COM_CloseNXT all        %prepares workspace
% h = COM_OpenNXT();      %look for USB devices
% COM_SetDefaultNXT(h);   %sets default handle

[cmPerDeg, movementNoise, degPerDeg, turningNoise] = NXTFullCalibration(1, 500, 50, 1500, 50);

pause(.5)

howFar = 10;
NXTMove (howFar, cmPerDeg, 50);

pause(.5)

howTurned = 50;
NXTTurn (howTurned, degPerDeg, 50);

pause(.5)

scan = NXTUltraScan(72, round(360/degPerDeg), 'Left', 50);
