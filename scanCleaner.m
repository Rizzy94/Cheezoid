clf('reset'); %resets figures 
clc;        %clears console
clear all;      %clears workspace
close all; %clears figures

% Load the example data.
load('exampleScans.mat')
% 
% data1 = 
% 
% fprintf('-->%s\n','Start training first ANFIS network. It may take one minute depending on your computer system.')
% tic
% anfis1 = anfis(data1, 4, [350,0,.001,.9,1.1], [0,0,0,0]); % train first ANFIS network
% toc

figure
plot(scanA(:,2), scanA(:,1));
hold on
plot(scanB(:,2), scanB(:,1));
plot(scanC(:,2), scanC(:,1));
plot(scanD(:,2), scanD(:,1));
plot(scanE(:,2), scanE(:,1));
plot(scanF(:,2), scanF(:,1));
legend('scanA', 'scanB', 'scanC', 'scanD', 'scanE', 'scanF');
xlim([0 360]);

hold off

