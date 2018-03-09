function [scan] = NXTUltraScan (numScans)

turnLimit = 360;            % in degrees

if 360/numScans ~= round(360/numScans)
    print('360/numScans must be an integer!')
    return
end

portScanner = 'SENSOR_1';   % which port is the ultrasound scanner in
portMotor = 'A';            % which port is the motor in

OpenUltrasonic(portScanner); %open usensor on port 4

turnPower = 50;

scan = zeros(round(360/numScans), 1);

for i = 1:round(360/numScans)
    
    scan(i) = GetUltrasonic(SENSOR_1); %get reading in “cm”
    mA = NXTMotor(portMotor, 'Power', turnPower, 'TachoLimit', 360/numScans);
    mA.SendToNXT();
    WaitFor(mA, 5)
    
end





