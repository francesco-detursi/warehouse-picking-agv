function checks = validateLayout(map,cfg)
labels = strings(0,1); passed = false(0,1); details = strings(0,1);
radius = cfg.robot.radius + cfg.robot.margin;
b = map.bounds;
objects = [map.obstacles;vertcat(map.docks.zone)];
inside = all(objects(:,1:2)>=b(1:2),'all') && ...
    all(objects(:,1:2)+objects(:,3:4)<=b(1:2)+b(3:4),'all');
record("Layout inside walls",inside,"Physical racks and marked operating zones");
separate = true;
for a = 1:size(objects,1)
    for c = a+1:size(objects,1)
        separate = separate && ~warehouse.geometry.rectanglesOverlap(objects(a,:),objects(c,:));
    end
end
record("No overlapping areas",separate,"Racks and all loading, unloading and waiting zones");
ids = [map.docks.id];
record("Unique station identifiers",numel(unique(ids))==numel(ids),"P01-P06, D and W");
record("One loading dock per rack",numel(map.rackIds)==sum([map.docks.kind]=="loading") && ...
    isequal(sort([map.docks(1:6).rackId]'),sort(map.rackIds)),"Fixed loading dock linked to each rack");
origin = map.docks(map.waitingIndex).pose(1:2);
for k = 1:numel(map.docks)
    dock = map.docks(k);
    p = warehouse.geometry.robotPolygon(dock.pose,cfg);
    z = dock.zone;
    fits = all(p>=z(1:2),'all') && all(p<=z(1:2)+z(3:4),'all');
    clearance = warehouse.geometry.footprintClearance(dock.pose,map,cfg);
    record(dock.id+": parked footprint",fits && clearance>=cfg.robot.margin, ...
        compose("Surface clearance %.3f m",clearance));
    turn = all(warehouse.geometry.isFree([dock.pose(1:2);dock.approach],map,radius));
    record(dock.id+": turning space",turn,"Circumscribed disk plus design margin at dock and approach");
    v = dock.pose(1:2)-dock.approach;
    heading = [cos(dock.pose(3)),sin(dock.pose(3))];
    aligned = dot(v,heading)>0 && abs(v(1)*heading(2)-v(2)*heading(1))<1e-9;
    access = warehouse.geometry.segmentFree(dock.approach,dock.pose(1:2),map, ...
        radius,cfg.validation.sampleSpacing);
    record(dock.id+": straight docking access",aligned && access,"Forward approach along docking heading");
    route = warehouse.planning.planRoute(origin,dock.approach,map,cfg);
    reachable = ~isempty(route);
    for j = 1:size(route,1)-1
        reachable = reachable && warehouse.geometry.segmentFree(route(j,:),route(j+1,:), ...
            map,radius,cfg.validation.sampleSpacing);
    end
    record(dock.id+": connected to W",reachable,"Static geometric route; not a simulated mission");
end
checks = table(labels,passed,details,'VariableNames',{'Check','Passed','Details'});
    function record(label,ok,detail)
        labels(end+1,1) = label;
        passed(end+1,1) = logical(ok);
        details(end+1,1) = detail;
    end
end
