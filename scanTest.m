clf('reset'); %resets figures 
clc;        %clears console
clear all;      %clears workspace
close all; %clears figures

nxt = Robot(); %creates robot object
nxt.beep(440, 200); %Beep beep

%try block means the robot will always be able to exit using nxt.close() even if there is an
%error in the code
try
    nxt.sensorAngle()
    numScans = 30;
    scanA = nxt.rotScan(numScans);
    pause(0.3)
    scanB = nxt.rotScan(numScans);  
    pause(0.3)

    figure
    plot(linspace(0,2*pi,numScans),scanA(), '-*')
    hold on
    plot(linspace(0,2*pi,numScans),scanB(), '-*')
    legend('scan1', 'scan2');
    hold off

catch ME
    warning('There was an error. Closing Robot Connection')
    display(ME.message) %print the error message
end
nxt.close();
