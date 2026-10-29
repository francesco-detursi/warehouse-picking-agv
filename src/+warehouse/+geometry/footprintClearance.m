function distance = footprintClearance(pose,map,cfg)
p = warehouse.geometry.robotPolygon(pose,cfg);
b = map.bounds;
distance = min([p(:,1)-b(1);b(1)+b(3)-p(:,1);p(:,2)-b(2);b(2)+b(4)-p(:,2)]);
for k = 1:size(map.obstacles,1)
    r = map.obstacles(k,:);
    q = [r(1) r(2);r(1)+r(3) r(2);r(1)+r(3) r(2)+r(4);r(1) r(2)+r(4)];
    axes = [1 0;0 1;cos(pose(3)) sin(pose(3));-sin(pose(3)) cos(pose(3))];
    separated = false;
    for j = 1:4
        pp = p*axes(j,:)'; qq = q*axes(j,:)';
        separated = separated || max(pp)<min(qq) || max(qq)<min(pp);
    end
    if ~separated, distance = min(distance,0); continue; end
    for j = 1:4
        next = mod(j,4)+1;
        distance = min(distance,min(pointSegment(p,q(j,:),q(next,:))));
        distance = min(distance,min(pointSegment(q,p(j,:),p(next,:))));
    end
end
end

function d = pointSegment(p,a,b)
v = b-a;
t = min(1,max(0,((p-a)*v')/(v*v')));
d = vecnorm(p-a-t.*v,2,2);
end
