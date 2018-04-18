function plotScan(scans)
    numScans = size(scans,1);
    for i = 1:size(scans,2)
        plot(linspace(0,2*pi,numScans),scans(:,i), '-*')
        legend(string(i))
    end
end