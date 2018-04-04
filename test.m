clf('reset');
clc;        %clears console
clear all;      %clears workspace
%% init
nxt = Robot(); %creates robot object
nxt.beep(440, 200);

%% scan
scansA = nxt.rotScan(30,80);
pause(1)
scansB = nxt.rotScan(30,80);
scansB = flip(scansB);
%% plotting
numScans = size(scansA, 1);
polarplot(linspace(0,2*pi,numScans),scansA(:,1), '-*')
hold on
polarplot(linspace(0,2*pi,numScans),scansB(:,1), '-*')
hold off

%% exit
nxt.close();