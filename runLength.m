function run = runLength(scan, n)
    array = [1 1 0 0 1 1 1 0 0 0 1 1];
    n = 11;
    run = 0;
    while true
        if array(n)
            run = run +1;
            n = mod(n +1, length(array));
            if n == 0
                n = length(array);
            end
        else
            break
        end
    end
end
