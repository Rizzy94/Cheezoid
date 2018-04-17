clear;
% Load the example data.
load('exampleScans2.mat', 'scanA', 'scanB', 'scanC', 'scanD', 'scanE', 'scanF');
% 
data1 = [scanA; scanB];% scanC; scanD; scanE; scanF];

genOpt = genfisOptions('GridPartition');
genOpt.NumMembershipFunctions = 10;
genOpt.InputMembershipFunctionType = 'trimf';
inFIS = genfis(data1(:,2),data1(:,1),genOpt);

opt = anfisOptions('InitialFIS',inFIS, 'EpochNumber',250);
opt.DisplayANFISInformation = 0;
opt.DisplayErrorValues = 0;
opt.DisplayStepSize = 0;
opt.DisplayFinalResults = 0;

tic
outFIS = anfis([data1(:,2) data1(:,1)],opt);
toc
% fprintf('-->%s\n','Start training first ANFIS network. It may take one minute depending on your computer system.')
% tic
% anfis1 = anfis(, 50, [350,0,.001,.9,1.1], [0,0,0,0]); % train first ANFIS network
% toc

X = [5:5:360]';
Y = evalfis(X, outFIS);


figure
plot(scanA(:,2), scanA(:,1));
hold on
plot(scanB(:,2), scanB(:,1));
plot(scanC(:,2), scanC(:,1));
plot(scanD(:,2), scanD(:,1));
plot(scanE(:,2), scanE(:,1));
plot(scanF(:,2), scanF(:,1));
plot(X, Y)
% legend('scanA', 'scanB', 'scanC', 'scanD', 'scanE', 'scanF', 'anfis');
xlim([0 360]);

hold off

