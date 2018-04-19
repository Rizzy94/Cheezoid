function [ang, oScans] = orthoScans(scan)
    numScans = size(scan,1);
    
    if mod(numScans, 4) ~= 0
        warning('Number of scans not divisable by 4')
    end
    %how does this cope when theres multiple ones of the same dist, it should
    %probably pick the middle one in that sequence.

    [ang, minInd] = orthoAngle(scan); %from kipps orthoAngle NEED TO ROUND
    ind1 = floor(minInd);
    indices = zeros(numScans,1);
    for i = 1:4
       indices(i) = mod(ind1+(i*(numScans/4)), numScans);
    end
    oScans = [scan(indices(1)); scan(indices(2)); scan(indices(3)); scan(indices(4))]; 

end