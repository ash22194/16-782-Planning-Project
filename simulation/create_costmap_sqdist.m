function [ cost_map ] = create_costmap_sqdist( map, epsilon )
%CREATE_COSTMAP_SQDIST Create a cost map that computes squared distance to
%obstacles (linear distance when inside)
%   map: environment map struct
%   epsilon: horizon upto which distance is computed
%   cost_map: cost_map where each coordinate maps to the cost

map = double(map);
ma = max(max(map));
map = map/ma;

Dint = double(-bwdist(map));
Dext = double(bwdist(1-map));

Cint = -Dint;
Cext = (1.0/(2.0*epsilon))*((min(Dext-epsilon, 0)).^2);

cost_map = Cint + Cext;

end

