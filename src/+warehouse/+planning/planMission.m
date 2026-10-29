function plan = planMission(state,map,cfg)
request=warehouse.mission.createRequest(state,map);
rackIndices=state.rackIndices;
dockIndices=zeros(size(rackIndices));
for k=1:numel(rackIndices)
    dockIndices(k)=find([map.docks.rackId]==map.rackIds(rackIndices(k)),1);
end
w=map.waitingIndex; d=map.dispatchIndex;
network=warehouse.planning.stationRoutes(map,cfg,unique([dockIndices w d]));
[best,bestDistance]=warehouse.planning.chooseVisitOrder(network.cost,dockIndices,w,d,w);
manual=[w dockIndices d w];
manualDistance=sum(network.cost(sub2ind(size(network.cost),manual(1:end-1),manual(2:end))));
if state.orderMode=="optimised", visits=best; else, visits=dockIndices; end
assert(~isempty(visits),'Warehouse:UnreachableMission','No feasible complete mission for the selected racks.');
sequence=[w visits d w];
legs=repmat(struct('from',"",'to',"",'path',zeros(0,2),'distance_m',0,'targetPose',zeros(1,3)),numel(sequence)-1,1);
route=zeros(0,2);
for k=1:numel(legs)
    a=sequence(k); b=sequence(k+1);
    path=network.routes{a,b};
    assert(~isempty(path),'Warehouse:UnreachableMission','No feasible route from %s to %s.',map.docks(a).id,map.docks(b).id);
    legs(k)=struct('from',map.docks(a).id,'to',map.docks(b).id,'path',path, ...
        'distance_m',network.cost(a,b),'targetPose',map.docks(b).pose);
    if isempty(route), route=path; else, route=[route;path(2:end,:)]; end %#ok<AGROW>
end
actualRacks=zeros(size(visits));
for k=1:numel(visits), actualRacks(k)=find(map.rackIds==map.docks(visits(k)).rackId,1); end
plan.version="phase4-smooth";
plan.status="planned";
plan.request=request;
plan.rackIndices=actualRacks;
plan.rackIds=reshape(map.rackIds(actualRacks),1,[]);
plan.stationIds=[map.docks(sequence).id];
plan.legs=legs;
plan.path=route;
plan.distance_m=sum([legs.distance_m]);
plan.requestedOrderDistance_m=manualDistance;
plan.bestOrderDistance_m=bestDistance;
plan.distanceCriterion="Minimum sum of precomputed directed smoothed station-route lengths; includes docking, D and return to W.";
end
