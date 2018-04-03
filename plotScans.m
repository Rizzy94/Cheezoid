function plotScans(scan)

numScans = size(scan, 1);
polarplot(linspace(0,2*pi,numScans),scan, '-*')

end