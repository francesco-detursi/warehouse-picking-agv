function free = segmentFree(a, b, map, radius, spacing)
distance = norm(b-a);
n = max(1,ceil(distance/spacing));
t = (0:n)'/n;
points = a + t.*(b-a);
% Half-step padding bounds unsampled distances along the segment.
free = all(warehouse.geometry.isFree(points,map,radius+distance/(2*n)));
end
