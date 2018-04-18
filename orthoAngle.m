function angle = orthoAngle(scan)

scanAux = scan;
scanAux(scan < 10) = 255;

[minVal, minInd] = min(scanAux);
numScan = length(scanAux);

angle = ((minInd-1)/numScan)*(2*pi);
if angle > pi
   angle = angle - 2*pi; 
end
%angleDeg = radtodeg(angle);

end