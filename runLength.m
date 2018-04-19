function run = runLength(array, n)
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
