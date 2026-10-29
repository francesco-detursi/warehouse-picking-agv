function [route,reason]=avoidPeople(pose,target,map,cfg,people,velocity,visible,scenario,settings)
route=zeros(0,2);reason="no_stationary_detection";
selected=visible(:) & vecnorm(velocity,2,2)<=settings.stationarySpeed;
if ~any(selected),return;end
 temporary=map;
padding=scenario.radius+max(0,scenario.separation-cfg.robot.margin)+settings.extraMargin;
xy=people(selected,:);
temporary.obstacles=[map.obstacles;xy-padding repmat(2*padding,size(xy,1),2)];
radius=cfg.robot.radius+cfg.robot.margin;
if ~warehouse.geometry.isFree(pose(1:2),temporary,radius)
 reason="insufficient_departure_space";return
end
if ~warehouse.geometry.segmentFree(target.approach,target.pose(1:2),temporary,radius,cfg.validation.sampleSpacing)
 reason="station_access_occupied";return
end
route=warehouse.planning.planRoute(pose(1:2),target.approach,temporary,cfg);
if isempty(route),reason="no_alternative";return;end
route=[route;target.pose(1:2)];
route=route([true;vecnorm(diff(route),2,2)>1e-9],:);
if size(route,1)<2,route=zeros(0,2);reason="already_at_target";return;end
route=warehouse.planning.smoothRoute(route,temporary,cfg);
reason="accepted";
end
