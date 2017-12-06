function cost = computeFinalCost(path,map)
    start = path(1,:);
    [x_size,y_size] = size(map);
    gridpath = round(path);
    gridpath(gridpath(:,1) == 0,1) = 1;
    gridpath(gridpath(:,1) > y_size,1) = y_size;
    gridpath(gridpath(:,2) == 0,2) = 1;
    gridpath(gridpath(:,2) > x_size,2) = x_size;
    path = [start;path];
    % Operating under the assumption that first column in path is column indices 
    % and the second column is row indices 
    sub = sub2ind(size(map),gridpath(:,2),gridpath(:,1));
    c = map(sub);
    value = (1+c/255);
    cost = sum(sqrt(sum(diff(path,1).^2,2)).*value);
end