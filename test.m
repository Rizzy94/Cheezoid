clf('reset');
clc;        %clears console
clear all;      %clears workspace

nxt = Robot(); %creates robot object
nxt.beep(440, 200); %Beep beep

%try block means the robot will always be able to exit using nxt.close() even if there is an
%error in the code
try
    scansA = nxt.rotScan(30)
    pause(1)
    scansB = nxt.rotScan(30)


    numScans = size(scansA, 1);
    polarplot(linspace(0,2*pi,numScans),scansA(:,1), '-*')
    hold on

    polarplot(linspace(0,2*pi,numScans),scansB(:,1), '-*')

    hold off
%     nxt.move(5)
%     nxt.turn(-40)
catch ME
    warning('There was an error. Closing Robot Connection')
    ME %print the error message
end
nxt.close();
