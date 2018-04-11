clf('reset');
clc;        %clears console
clear all;      %clears workspace

nxt = Robot(); %creates robot object
nxt.beep(440, 200); %Beep beep

%try block means the robot will always be able to exit using nxt.close() even if there is an
%error in the code
try
    nxt.setUpScanner();
    nxt.sensorAngle()
    numScans = 72;
    scanA = nxt.rotScan(numScans)
    scanB = nxt.rotScan(numScans)
    polarplot(linspace(0,2*pi,numScans),scanA(:,1), '-*')
    hold on
    polarplot(linspace(0,2*pi,numScans),scanB(:,1), '-*')
    legend('scan1', 'scan2');
    hold off
catch ME
    warning('There was an error. Closing Robot Connection')
    display(ME.message) %print the error message
end
nxt.close();
