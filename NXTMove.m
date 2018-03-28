function NXTMove (howFar, cmPerDeg, movePow)

mAB = NXTMotor('AB', 'Power', movePow, 'TachoLimit', round(howFar/cmPerDeg));
mAB.SpeedRegulation = false;
mAB.SmoothStart = true;
mAB.SendToNXT();
mAB.WaitFor();
mAB.Stop('brake') 

end
