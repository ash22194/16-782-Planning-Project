function cost = computeFinalCost(path,map)
    start = path(1,:);
    [x_size,y_size] = size(map);
    gridpath = round(path);
    gridpath(gridpath(:,1) == 0,1) = 1;
    gridpath(gridpath(:,1) > y_size,1) = y_size;
    gridpath(gridpath(:,2) == 0,2) = 1;
    gridpath(gridpath(:,2) > x_size,2) = x_size;
    path = [start;path];
    dist = sqrt(sum(diff(path,1).^2,2));
    % Operating under the assumption that first column in path is column indices 
    % and the second column is row indices 
    sub = sub2ind(size(map),gridpath(:,1),gridpath(:,2));
    c = map(sub);
    blocked = sum(c==255)>0;
    if (blocked)
        cost = inf;
    else
        value = (1+c/255);
        cost = sum(dist.*value);
    end
    
end