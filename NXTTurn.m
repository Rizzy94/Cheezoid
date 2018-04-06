function NXTTurn (howTurned, degPerDeg, turnPow)

mA = NXTMotor('A', 'Power', turnPow, 'TachoLimit', round((howTurned*180/pi)/degPerDeg));
mB = NXTMotor('B', 'Power', -turnPow, 'TachoLimit', round((howTurned*180/pi)/degPerDeg));
mB.SpeedRegulation = false;
mB.SmoothStart = true;
mA.SpeedRegulation = false;
mA.SmoothStart = true;

mA.SendToNXT();
mB.SendToNXT();
mA.WaitFor();
mB.WaitFor();
mA.Stop('brake') 
mB.Stop('brake') 

end