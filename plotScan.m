function plotScan(scans)
    numScans = size(scans,1);
    figure
    hold on
    for i = 1:size(scans,2)
        plot(linspace(0,2*pi,numScans),scans(:,i), '-*')
    end
    xlabel('Angle (rad)')
    ylabel('Distance (cm)')
    hold off
end