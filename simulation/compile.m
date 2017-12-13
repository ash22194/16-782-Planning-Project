function compile(ada)
    commandADA();
    
    if ~ada
        commandCHOMP();
        commandRRT();
    end
end