function [angle, minInd] = orthoAngle(scan)

scanAux = scan;
scanAux(scan < 10) = 255;

auxInd = (scanAux == min(scanAux));
if sum(auxInd) == 1                     %if the minimum is unique do this
    [minVal, minInd] = min(scanAux);
    numScan = length(scanAux);

    angle = ((minInd-1)/numScan)*(2*pi);
    if angle > pi
       angle = angle - 2*pi; 
    end
end
if sum(auxInd) > 1
    [ind, runLength] = longestRun(auxInd);
    minInd = mod(ind + (runLength-1)/2, length(auxInd));
    numScan = length(scanAux);

    angle = ((minInd-1)/numScan)*(2*pi);
    if angle > pi
       angle = angle - 2*pi; 
    end
end
    

%angleDeg = radtodeg(angle);

end