clf('reset'); %resets figures 
clc;        %clears console
clear all;      %clears workspace
close all; %clears figures


load('exampleScans.mat', 'scanA', 'scanB', 'scanC', 'scanD', 'scanE', 'scanF')

scanA = scanA(:,1);
scanB = scanB(:,1);
scanC = scanC(:,1);
scanD = scanD(:,1);
scanE = scanE(:,1);
scanF = scanF(:,1);

save('exampleScans.mat', 'scanA', 'scanB', 'scanC', 'scanD', 'scanE', 'scanF')
plotScan([scanA scanB scanC scanD scanE scanF])