function viable = safePath(bot, from, to)
    numChecks = 50;
    wallDist = 10;
    viable = 1;
    lineX = linspace(from(1),to(1),numChecks);
    lineY = linspace(from(2),to(2),numChecks);

    for i = 1:numChecks
       bot.setBotPos([lineX(i), lineY(i)]);
       if min(bot.ultraScan())<wallDist
           viable = 0;
           break  
       end
    end
end