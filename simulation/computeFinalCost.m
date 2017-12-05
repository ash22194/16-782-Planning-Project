function cost = computeFinalCost(path,map)
    start = path(1,:);
    gridpath = round(path+1);
    path = [start;path];
    % Operating under the assumption that first column in path is column indices 
    % and the second column is row indices 
    sub = sub2ind(size(map),gridpath(:,2),gridpath(:,1));
    c = map(sub);
    value = (1+c/255);
    cost = sum(sqrt(sum(diff(path,1).^2,2)).*value);
end