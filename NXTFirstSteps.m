COM_CloseNXT all        %prepares workspace
h = COM_OpenNXT();      %look for USB devices
COM_SetDefaultNXT(h);   %sets default handle

NXT_PlayTone(440, 500); %(freq,duration)

%COM_CloseNXT(h);

%% Touch Sensor Reading

OpenSwitch(SENSOR_3); %open push sensor on port 3
GetSwitch(SENSOR_3) %get reading 1 or 0

%% Ultrasound Sensor Reading

OpenUltrasonic(SENSOR_1); %open usensor on port 4
GetUltrasonic(SENSOR_1) %get reading in “cm”

% WHEN DONE WITH ULTRASOUND WE CLOSE IT

CloseSensor(SENSOR_4); 

%% Motor Action

mA = NXTMotor('A') %motor connected to port A
mB = NXTMotor('B')
mB.Power = 25
mA.Power = -25; %select power output [-100,100]
mA.SendToNXT(); %move motor
mB.SendToNXT();

mA.Stop('brake') %stops and holds position
mA.Stop('off') %stops and coasts
mB.Stop('off')

mA.TachoLimit = 360; %in degrees
mA = NXTMotor('A', 'Power', -60, 'TachoLimit', 1000);

mA = NXTMotor('A', 'Power', -50, 'TachoLimit', 360);
mA.SendToNXT();
mA.WaitFor(); % without this the motor will barely move!
mA.Stop('off');
data = mA.ReadFromNXT() %reads state of motor