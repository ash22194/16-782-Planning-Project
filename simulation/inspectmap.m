function [] = inspectmap(filename, radius)

map = load(filename);

if floor(radius) == radius && radius >= 1
    envmap = map;
    envmap(envmap<255) = 0;
    bogmap = robotics.BinaryOccupancyGrid(envmap);
    inflate(bogmap, radius);
    map(occupancyMatrix(bogmap)) = 255;
end

% make the color consistent with robotics.BinaryOccupancyGrid
% black - high cost/occupied
map = 255 - map;

close all;

%draw the environment
image(map);
colormap(gray(256));
axis image

end