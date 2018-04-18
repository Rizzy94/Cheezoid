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
%     numScans = 72;
%     scanA = nxt.rotScan(numScans);
%     pause(0.3)
%     scanB = nxt.rotScan(numScans);  
%     pause(0.3)
% % 
% % %     figure
% % %     polarplot(linspace(0,2*pi,numScans),scanA(:,1), '-*')
% % %     hold on
% % %     polarplot(linspace(0,2*pi,numScans),scanB(:,1), '-*')
% % %     legend('scan1', 'scan2');
% % %     hold off
% 
% close all
% clf
% 
%     map = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]; %default map
%     target = [80,80];
% 
%     startAngle =0;  
%     endAngle = ((numScans-1)*2*pi)/numScans;  
%     angles = (startAngle:(endAngle - startAngle)/(numScans-1):endAngle);
%     scanLines =  [cos(angles); sin(angles)]'*100;
%     scanOffSet = [0, 0];
% 
%     plotMe = true;
% 
%     botGhost = BotSim(map); %how to set up a vector of objects
%     botGhost.setScanConfig(scanLines,scanOffSet);
%     botGhost.setSensorNoise(0);
%     botGhost.setMotionNoise(0);
%     botGhost.setTurningNoise(0);
%     botGhost.setBotPos([21,65.5]); %spawn the particles in random locations
%     botGhost.setBotAng(-pi/2);
%           
%     scanGhost = botGhost.ultraScan();
%     scanGhost = circshift(scanGhost, -1);
%     
%     figure
%     plot(scanA(:,2), scanA(:,1));
%     hold on
%     plot(scanB(:,2), scanB(:,1));
%     plot(linspace(0,360,numScans), flipud(scanGhost), 'LineWidth',2)
%     hold off
%     legend('scanA', 'scanB', 'flipud(scanGhost)');
%     mario(nxt)

catch ME
    warning('There was an error. Closing Robot Connection')
    display(ME.message) %print the error message
end
% nxt.turn(-pi/4)
nxt.callibrate(2)
nxt.close();
