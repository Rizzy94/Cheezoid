clf('reset'); %resets figures 
clc;        %clears console
clear all;      %clears workspace
close all; %clears figures

% Load the example data.
load('exampleScans.mat')
scans = [scanA; scanB; scanC; scanD; scanE; scanF];

data1 = [scans(:,2) scans(:,1)];

fprintf('-->%s\n','Start training first ANFIS network. It may take one minute depending on your computer system.')
tic
anfis1 = anfis(data1, 7, 250, [0,0,0,0]); % train second ANFIS network
toc

X = [5:5:360];

Y = evalfis(X, anfis1); % theta1 predicted by anfis1


figure
plot(scanA(:,2), scanA(:,1));
hold on
plot(scanB(:,2), scanB(:,1));
plot(scanC(:,2), scanC(:,1));
plot(scanD(:,2), scanD(:,1));
plot(scanE(:,2), scanE(:,1));
plot(scanF(:,2), scanF(:,1));
plot(X,Y)
legend('scanA', 'scanB', 'scanC', 'scanD', 'scanE', 'scanF', 'anfis');
xlim([0 360]);

hold off

