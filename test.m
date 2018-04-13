clf('reset'); %resets figures 
clc;        %clears console
clear all;      %clears workspace
close all; %clears figures

nxt = Robot(); %creates robot object
nxt.beep(440, 200); %Beep beep

%try block means the robot will always be able to exit using nxt.close() even if there is an
%error in the code
try
%     nxt.sensorAngle()
%     numScans = 40;
%     scanA = nxt.rotScan(numScans);
%     pause(0.3)
%     scanB = nxt.rotScan(numScans);  
%     pause(0.3)
%     scanC = nxt.rotScan(numScans);
%     pause(0.3)
%     scanD = nxt.rotScan(numScans);
%     figure
%     polarplot(linspace(0,2*pi,numScans),scanA(:,1), '-*')
%     hold on
%     polarplot(linspace(0,2*pi,numScans),scanB(:,1), '-*')
%     legend('scan1', 'scan2');
%     hold off
%     figure
%     plot(1:numScans, scanA);
%     hold on
%     plot(1:numScans, scanB);
%     plot(1:numScans, scanC);
%     plot(1:numScans, scanD);
%     hold off
%     legend('scan1', 'scan2', 'scan3', 'scan4');
    mario(nxt);
catch ME
    warning('There was an error. Closing Robot Connection')
    display(ME.message) %print the error message
end
nxt.close();
