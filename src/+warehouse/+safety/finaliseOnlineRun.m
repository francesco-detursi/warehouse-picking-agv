function run=finaliseOnlineRun(run,scenario,log,states)
trajectory=run.trajectory;row=height(trajectory);cfg=run.config;map=run.map;
ped=zeros(row,2*numel(scenario.people));seen=false(row,numel(scenario.people));
poses=trajectory{:, {'X_m','Y_m','Heading_rad'}};times=trajectory.Time_s;
for k=1:row
 if isfield(run,'pedestrians'),positions=reshape(run.pedestrians(k,:),2,[])';
 else,positions=warehouse.safety.pedestrianState(times(k),scenario);end
 ped(k,:)=reshape(positions',1,[]);
 seen(k,:)=warehouse.safety.visiblePedestrians(poses(k,:),positions,map,scenario.sensorRange)';
end
clearance=trajectory.Clearance_m;humanGap=inf(row,1);
c=cos(poses(:,3));s=sin(poses(:,3));
for person=1:numel(scenario.people)
 delta=ped(:,2*person-1:2*person)-poses(:,1:2);
 local=[c.*delta(:,1)+s.*delta(:,2),-s.*delta(:,1)+c.*delta(:,2)];
 out=max(abs(local)-[cfg.robot.length cfg.robot.width]/2,0);
 humanGap=min(humanGap,vecnorm(out,2,2)-scenario.radius);
end
bodyTravel=abs(diff(trajectory.Distance_m))+cfg.robot.radius*abs(diff(trajectory.Heading_rad));
pedTravel=zeros(row-1,1);
for j=1:numel(scenario.people),pedTravel=max(pedTravel,vecnorm(diff(ped(:,2*j-1:2*j)),2,2));end
humanBound=min(humanGap(1:end-1),humanGap(2:end))-(bodyTravel+pedTravel)/2;
staticBound=min(clearance(1:end-1),clearance(2:end))-bodyTravel/2;
assert(all(humanBound>=scenario.separation),'Warehouse:PedestrianSeparation','The pedestrian separation assumption was violated; this run is rejected.');
assert(all(staticBound>=cfg.robot.margin),'Warehouse:Collision','Static footprint margin violated.');
trajectory.Clearance_m=clearance;trajectory.PedestrianClearance_m=humanGap;
trajectory.TrackingError_m=hypot(trajectory.X_m-trajectory.ReferenceX_m,trajectory.Y_m-trajectory.ReferenceY_m);

rotation=trajectory.Phase=="rotating";
trajectory.SpeedScale=log(:,2);trajectory.SupervisorState=states;
trajectory.Phase(states=="hold")="pedestrian_hold";
trajectory.Phase(states=="slowing")="pedestrian_braking";
trajectory.Phase(states=="resuming")="pedestrian_restart";
trajectory.Phase(end)=run.status;
run.trajectory=trajectory;run.scenario=scenario;run.pedestrians=ped;run.detected=seen;
run.supervisor=table(log(:,1),log(:,2),states,logical(log(:,3)),humanGap, ...
 'VariableNames',{'Time_s','SpeedScale','State','ConflictDetected','PedestrianClearance_m'});
transitions=[true;states(2:end)~=states(1:end-1)];run.supervisorEvents=run.supervisor(transitions,:);
dt=[0;diff(trajectory.Time_s)];m=run.metrics;
m.TurningTime_s=sum(dt(rotation & (abs(trajectory.YawRate_radps)>1e-9)));
m.PedestrianHoldTime_s=sum(dt(states=="hold"));
m.PedestrianHoldEpisodes=sum(states=="hold" & [true;states(1:end-1)~="hold"]);
m.MinimumPedestrianClearance_m=min(humanGap);m.PedestrianClearanceBound_m=min(humanBound);
m.TravelAndTurningTime_s=m.TravelAndTurningTime_s-m.PedestrianHoldTime_s;
if isfield(scenario,'seed')
 m.TrafficSeed=scenario.seed;m.PedestrianCount=numel(scenario.people);
 m.PedestrianEmergencyStops=sum(run.crowdSummary.EmergencyStops);
 m.PedestrianDestinationsReached=sum(run.crowdSummary.DestinationsReached);
 pairGap=inf;rackGap=inf;
 for person=1:numel(scenario.people)
  xy=ped(:,2*person-1:2*person);
  stepTravel=vecnorm(diff(xy),2,2);
  for other=person+1:numel(scenario.people)
   otherXY=ped(:,2*other-1:2*other);
   gap=vecnorm(xy-otherXY,2,2)-2*scenario.radius;
   bound=min(gap(1:end-1),gap(2:end))-(stepTravel+vecnorm(diff(otherXY),2,2))/2;
   pairGap=min(pairGap,min(bound));
  end
  for obstacle=1:size(map.obstacles,1)
   box=map.obstacles(obstacle,:);
   delta=max(max(box(1:2)-xy,0),xy-box(1:2)-box(3:4));
   gap=vecnorm(delta,2,2)-scenario.radius;
   rackGap=min(rackGap,min(min(gap(1:end-1),gap(2:end))-stepTravel/2));
  end
 end
 assert(pairGap>=0 && rackGap>=0,'Warehouse:PedestrianCollision','Pedestrian geometry validation failed.');
 m.MinimumPedestrianPairGap_m=pairGap;m.MinimumPedestrianRackGap_m=rackGap;
end
run.metrics=m;run.motion.supervisorSampleTime=run.motion.sampleTime;
end
