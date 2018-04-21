function [nxtScan, botScan] = scanAll(nxt, bot, numScans)
    %1. scan using nxt robot
    rScan = nxt.rotScan(numScans);
    
    %2. figure out the orthogonal scans
    [ang, rScan] = orthoScans(rScan);
    
    %3. Ang gives one orthogonal scan angle, deduce them all:
    angles = (ang:pi/2:ang+(3*pi/2));
    angles = wrapTo2Pi(angles);
    
    %4. Set the botSim objects to scan at these angles
    scanLines =  [cos(angles); sin(angles)]'*100;
    scanOffSet = [0 0]; %offset in x,y from wheelbase
    botScan = zeros(length(bot), 1);
    for i = 1:length(bot)
        bot(i).setScanConfig(scanLines,scanOffSet);
        botScan(i) = bot(i).ultraScan;
    end
    
    nxtScan = rScan; %maaaayyybee, not sure if the vector is in right order vs botscan
end