function [ind, run] = longestRun(array)
    ind = 0;
    run = 0;
    for i = 1:length(array)
        currentRun = runLength(array, i);
        if currentRun > run
            run = currentRun;
            ind = i;
        end
    end    
end