function angle = orthoAngle(scan)

scanAux = scan;

[minVal, minInd] = min(scanAux);
numScan = length(scanAux);

angle = ((minInd-1)/numScan)*(2*pi);
%angleDeg = radtodeg(angle);

end