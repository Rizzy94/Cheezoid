clf('reset'); %resets figures 
clc;        %clears console
clear all;      %clears workspace
close all; %clears figures
pause(30)

nxt = Robot(); %creates robot object
nxt.beep(440, 200); %Beep beep
%try allows for safe closing of robot connection.
%TODO: catch ctrl-c
try
    
    
    
catch ME
    warning('There was an error. Closing Robot Connection')
    display(ME.message) %print the error message
end

nxt.close();
