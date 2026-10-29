function network = stationRoutes(map,cfg,indices)
persistent previousMap previousConfig cache
if isempty(cache) || ~isequal(previousMap,map) || ~isequal(previousConfig,cfg)
    previousMap=map; previousConfig=cfg;
    count=numel(map.docks);
    cache=struct('routes',{cell(count)},'cost',inf(count),'known',false(count));
end
radius=cfg.robot.radius+cfg.robot.margin;
for from=indices
    for to=indices
        if from==to || cache.known(from,to), continue; end
        target=map.docks(to);
        access=warehouse.geometry.segmentFree(target.approach,target.pose(1:2),map,radius,cfg.validation.sampleSpacing);
        route=zeros(0,2);
        if access
            route=warehouse.planning.planRoute(map.docks(from).pose(1:2),target.approach,map,cfg);
            if ~isempty(route)
                route=[route;target.pose(1:2)];
                route=route([true;vecnorm(diff(route),2,2)>1e-10],:);
                for k=1:size(route,1)-1
                    if ~warehouse.geometry.segmentFree(route(k,:),route(k+1,:),map,radius,cfg.validation.sampleSpacing)
                        route=zeros(0,2); break
                    end
                end
            end
        end
        if ~isempty(route),route=warehouse.planning.smoothRoute(route,map,cfg);end
        cache.routes{from,to}=route;
        if ~isempty(route), cache.cost(from,to)=sum(vecnorm(diff(route),2,2)); end
        cache.known(from,to)=true;
    end
end
network=cache;
end
