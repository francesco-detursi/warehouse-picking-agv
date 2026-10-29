function crowd=advanceCrowd(crowd,dt,robotPose,robotVelocity,map,cfg,scenario)
n=size(crowd.position,1);
for k=1:n
 p=crowd.position(k,:);old=crowd.velocity(k,:);
 if crowd.stall(k)>6
  crowd=warehouse.safety.redirectPedestrian(crowd,k,robotPose,map,cfg,scenario);
  crowd.stall(k)=0;
 end
 if crowd.pause(k)>0
  crowd.pause(k)=max(0,crowd.pause(k)-dt);crowd.velocity(k,:)=[0 0];continue
 end
 if isempty(crowd.route{k})
  current=crowd.target(k);choices=setdiff(1:size(scenario.nodes,1),current);
  goal=choices(randi(crowd.stream{k},numel(choices)));
  path=shortestpath(scenario.graph,current,goal);
  crowd.route{k}=path(2:end);crowd.target(k)=goal;
 end
 if isempty(crowd.detour{k}),waypoint=scenario.nodes(crowd.route{k}(1),:);
 else,waypoint=crowd.detour{k}(1,:);end
 delta=waypoint-p;distance=norm(delta);
 if distance<crowd.bestDistance(k)-0.05,crowd.bestDistance(k)=distance;crowd.stall(k)=0;
 else,crowd.stall(k)=crowd.stall(k)+dt;end
 if distance<0.06 && norm(old)<0.02
  crowd.bestDistance(k)=inf;crowd.stall(k)=0;
  if ~isempty(crowd.detour{k})
   crowd.detour{k}(1,:)=[];crowd.velocity(k,:)=[0 0];continue
  end
  crowd.route{k}(1)=[];
  if isempty(crowd.route{k})
   crowd.visits(k)=crowd.visits(k)+1;
   crowd.pause(k)=1+4*rand(crowd.stream{k});
  end
  crowd.velocity(k,:)=[0 0];continue
 end
 angle=atan2(delta(2),delta(1));speed=min(scenario.people(k).speed,sqrt(2*scenario.walkAcceleration*max(0,distance-0.02)));
 offsets=deg2rad([0 25 -25 50 -50 80 -80 115 -115 180])';
 candidates=speed*[cos(angle+offsets) sin(angle+offsets)];
 candidates=[candidates;0.5*candidates;0 0];
 change=candidates-old;factor=min(1,scenario.walkAcceleration*dt./max(vecnorm(change,2,2),eps));
 candidates=old+change.*factor;
 horizon=scenario.walkPrediction;valid=true(size(candidates,1),1);
 for h=[0.25 0.5 1]*horizon
  positions=p+candidates*h;
  valid=valid & all(positions>=map.bounds(1:2)+scenario.radius+0.1,2) & ...
      all(positions<=map.bounds(1:2)+map.bounds(3:4)-scenario.radius-0.1,2);
  for obstacle=1:size(map.obstacles,1)
   box=map.obstacles(obstacle,:);
   d=max(max(box(1:2)-positions,0),positions-box(1:2)-box(3:4));
   valid=valid & vecnorm(d,2,2)>scenario.radius+0.1;
  end
 end
 relative=p-robotPose(1:2);vel=candidates-robotVelocity;
 required=min(cfg.robot.radius+scenario.radius+scenario.robotGap,norm(relative)-1e-8);
 valid=valid & separation(relative,vel,horizon)>=required;
 for other=setdiff(1:n,k)
  relative=p-crowd.position(other,:);vel=candidates-crowd.velocity(other,:);
  required=min(2*scenario.radius+scenario.personalGap,norm(relative)-1e-8);
  valid=valid & separation(relative,vel,horizon)>=required;
 end
 score=vecnorm(p+candidates*0.7-waypoint,2,2)+0.08*vecnorm(candidates-old,2,2);
 score=score+0.15*(vecnorm(candidates,2,2)<0.01 & distance>0.2);
 score(~valid)=inf;[best,index]=min(score);
 if ~isfinite(best)
  chosen=old*max(0,1-scenario.walkAcceleration*dt/max(norm(old),eps));
  next=p+chosen*dt;
  safe=norm(next-robotPose(1:2))>=cfg.robot.radius+scenario.radius+scenario.separation+0.04;
  for other=setdiff(1:n,k)
   safe=safe && norm(next-crowd.position(other,:))>=2*scenario.radius+0.04+norm(crowd.velocity(other,:))*dt;
  end
  for obstacle=1:size(map.obstacles,1)
   box=map.obstacles(obstacle,:);d=max(max(box(1:2)-next,0),next-box(1:2)-box(3:4));
   safe=safe && norm(d)>scenario.radius+0.04;
  end
  if ~safe
   chosen=[0 0];
   if norm(old)>scenario.walkAcceleration*dt+1e-9,crowd.emergencyStops(k)=crowd.emergencyStops(k)+1;end
  end
  crowd.yieldTime(k)=crowd.yieldTime(k)+dt;
 else
  chosen=candidates(index,:);
  if norm(chosen)<0.1,crowd.yieldTime(k)=crowd.yieldTime(k)+dt;end
 end
 crowd.position(k,:)=p+chosen*dt;crowd.velocity(k,:)=chosen;
 crowd.distance(k)=crowd.distance(k)+norm(chosen)*dt;
end
end
function gap=separation(position,velocity,horizon)
time=max(0,min(horizon,-sum(position.*velocity,2)./max(sum(velocity.^2,2),eps)));
gap=vecnorm(position+velocity.*time,2,2);
end
