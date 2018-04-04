clc;        %clears console
clear all;      %clears workspace

nxt = Robot(); %creates robot object
nxt.beep(440, 200);
scans = nxt.rotScan(40,80)
plotScans(scans)
pause(1)
scans = flip(scans)
scans = nxt.rotScan(40,80)
plotScans(scans)


nxt.close();
