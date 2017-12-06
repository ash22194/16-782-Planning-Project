function indexLookAhead = computeLookAheadPoint(loc,path,distance)
    x = loc(1);
    y = loc(2);
    [N,~] = size(path);
    if (N==2)
        indexLookAhead = 2;
    else
        ref = ones(N,1)*[x,y];
        dist = sqrt(sum((path - ref).^2,2));
        candidates = (dist <= distance);
        indexLookAhead = find(diff(candidates,1),1);    
    end
    
end