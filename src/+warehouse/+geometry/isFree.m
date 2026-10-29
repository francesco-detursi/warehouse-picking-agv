function free = isFree(points, map, radius)
arguments
    points (:,2) double {mustBeFinite}
    map struct
    radius (1,1) double {mustBeNonnegative}
end
b = map.bounds;
free = points(:,1)>b(1)+radius & points(:,1)<b(1)+b(3)-radius & ...
    points(:,2)>b(2)+radius & points(:,2)<b(2)+b(4)-radius;
for k = 1:size(map.obstacles,1)
    r = map.obstacles(k,:);
    dx = max(max(r(1)-points(:,1),0),points(:,1)-r(1)-r(3));
    dy = max(max(r(2)-points(:,2),0),points(:,2)-r(2)-r(4));
    free = free & hypot(dx,dy)>radius;
end
end
