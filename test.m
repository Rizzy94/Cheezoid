clf('reset');
clc;        %clears console
clear all;      %clears workspace

nxt = Robot(); %creates robot object
nxt.beep(440, 200); %Beep beep

%try block means the robot will always be able to exit using nxt.close() even if there is an
%error in the code
try
%     scansA = nxt.rotScan(15)
%     pause(1)
%     scansB = nxt.rotScan(15)
% 
% 
%     numScans = size(scansA, 1);

% 
%     hold off
%     nxt.move(5)
%     nxt.turn(pi)
%     nxt.turn(-pi)
%     nxt.move(-5)
    nxt.setUpScanner();
    nxt.sensorAngle();
    numScans = 30
    scanA = nxt.rotScan(numScans)
    scanB = nxt.rotScan(numScans)
    polarplot(linspace(0,2*pi,numScans),scanA(:,1), '-*')
    hold on
    polarplot(linspace(0,2*pi,numScans),scanB(:,1), '-*')
    legend('scan1', 'scan2');
    hold off
catch ME
    warning('There was an error. Closing Robot Connection')
    ME %print the error message
end
nxt.close();
