function crowd=redirectPedestrian(crowd,k,robot,map,cfg,scenario)
p=crowd.position(k,:);
choices=find(vecnorm(scenario.nodes-robot(1:2),2,2)>cfg.robot.radius+scenario.radius+scenario.robotGap);
choices=choices(randperm(crowd.stream{k},numel(choices)));
walkCfg=cfg;walkCfg.robot.radius=scenario.radius;walkCfg.robot.margin=0.05;
walkCfg.planner.resolution=0.5;
body=warehouse.geometry.robotPolygon(robot,cfg);low=min(body,[],1);high=max(body,[],1);
walkMap=map;walkMap.obstacles=[map.obstacles;low high-low];
for j=1:min(3,numel(choices))
 goal=choices(j);
 route=warehouse.planning.planRoute(p,scenario.nodes(goal,:),walkMap,walkCfg);
 if size(route,1)<2,continue;end
 crowd.detour{k}=route(2:end-1,:);crowd.route{k}=goal;crowd.target(k)=goal;
 crowd.bestDistance(k)=inf;crowd.redirections(k)=crowd.redirections(k)+1;
 return
end
end
