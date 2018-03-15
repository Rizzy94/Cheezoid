% This is an example for scanning the environment.
% The scan sensor needs to be coupled with the motor.
% Before you run this, make sure that 
%- the touch sensor is plugged to Port 1
%- the ultrasound sensor is plugged to Port 4
%- a motor is plugged to Port C
%- the robot is connected via USB and you are running motorControl22 on it
%
% lets play...

%An example of the ultrasound scanning function
clf;        %clears figures
clc;        %clears console
clear;      %clears workspace

COM_CloseNXT all;  
h = COM_OpenNXT();
COM_SetDefaultNXT(h);

OpenUltrasonic(SENSOR_4); %open usensor on port 4
mot = NXTMotor('C');  %motor connected to port C
mot.ResetPosition();  %only do this once at the start
mot.SmoothStart = 1;

%performs a 360 degree scan at 20% power with plotting on
[rad ang] = ultraScan(mot,20,6)

% plots the results
polar(ang/(360)*2*pi,rad,'r');

%% exit
COM_CloseNXT(h); %clean before program exit 