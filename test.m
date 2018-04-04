clc;        %clears console
clear all;      %clears workspace

nxt = Robot(); %creates robot object
nxt.beep(440, 200);
scans = nxt.scan(10)
nxt.beep(440, 200);

nxt.close();
