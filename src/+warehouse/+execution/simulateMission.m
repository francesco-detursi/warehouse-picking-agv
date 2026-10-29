function run = simulateMission(plan,map,cfg,motion,scenario)
if nargin<4,motion=warehouse.config.motionSettings();end
online=nargin>=5;
randomTraffic=online && isfield(scenario,'kind') && scenario.kind=="random";
if randomTraffic
 crowd=warehouse.safety.initialiseCrowd(scenario);
 pedestrianLog=reshape(crowd.position',1,[]);
end
scale=1;clearSince=-inf;zoneOccupied=false;conflict=false;superState="clear";
superLog=[0 1 0];superStates="clear";
futurePath=plan.path;
replanning=warehouse.config.replanningSettings();
if online && isfield(scenario,'replanning'),replanning=scenario.replanning;end
holdSince=NaN;lastAttempt=-inf;rerouted=false;replacement=zeros(0,2);
replanLog=struct('Time_s',{},'Target',{},'Outcome',{},'Length_m',{});
routeUpdates=struct('time_s',{},'leg',{},'path',{});
assert(plan.status=="planned",'Warehouse:InvalidPlan','A complete plan is required.');
validateattributes([motion.loadingTime motion.unloadingTime],{'numeric'},{'finite','nonnegative'});
pose=map.docks(map.waitingIndex).pose;
assert(norm(plan.path(1,:)-pose(1:2))<1e-9,'Warehouse:InvalidStart','Mission must start at W.');
if isfield(motion,'initialPoseOffset')
    validateattributes(motion.initialPoseOffset,{'numeric'},{'size',[1 3],'finite'});
    assert(norm(motion.initialPoseOffset(1:2))<=0.02 && abs(motion.initialPoseOffset(3))<=deg2rad(1), ...
        'Warehouse:InitialOffset','Initial perturbation exceeds the verification range.');
    pose=pose+motion.initialPoseOffset;
end
time=0; distance=0; completed=0;terminated=false;
data=[0 pose 0 0 0 0 0];
phases="waiting"; targets="W";references=pose(1:2);
events=struct('type',{},'station',{},'time_s',{},'x_m',{},'y_m',{},'heading_rad',{},'completedPicks',{});
for leg=1:numel(plan.legs)
    item=plan.legs(leg); target=item.to;
    assert(norm(item.path(1,:)-pose(1:2))<=motion.positionTolerance, ...
        'Warehouse:DisconnectedLeg','A mission leg is disconnected.');
    rerouted=true;
    while rerouted
    rerouted=false;
    angles=unwrap(atan2(diff(item.path(:,2)),diff(item.path(:,1))));
    breaks=[1;find(abs(diff(angles))>deg2rad(12))+1;size(item.path,1)];
    for section=1:numel(breaks)-1
        path=item.path(breaks(section):breaks(section+1),:);
        ref=warehouse.execution.routeReference(path,motion);
        futurePath=path;
        rotateTo(ref(1,4));
        if terminated || rerouted,break;end
        if online
            followReference(ref,"travelling");
            if terminated || rerouted,break;end
        else
        for k=2:size(ref,1)
            dt=ref(k,1)-ref(k-1,1);
            delta=ref(k-1,2:3)-pose(1:2);
            ex=cos(pose(3))*delta(1)+sin(pose(3))*delta(2);
            ey=-sin(pose(3))*delta(1)+cos(pose(3))*delta(2);
            et=atan2(sin(ref(k-1,4)-pose(3)),cos(ref(k-1,4)-pose(3)));
            gains=motion.trackingGains;
            desiredV=ref(k,5)*cos(et)+gains(1)*ex;
            desiredW=ref(k,6)+gains(2)*ref(k,5)*ey+gains(3)*sin(et);
            oldV=data(end,5);oldW=data(end,6);
            v=max(0,min(motion.maxSpeed,desiredV));
            v=max(oldV-motion.deceleration*dt,min(oldV+motion.acceleration*dt,v));
            w=max(-motion.maxYawRate,min(motion.maxYawRate,desiredW));
            w=max(oldW-motion.yawAcceleration*dt,min(oldW+motion.yawAcceleration*dt,w));
            moveStep(dt,v,w,ref(k,2:3),"travelling");
        end
        end
        while abs(data(end,5))+abs(data(end,6))>1e-10
            dt=motion.sampleTime;
            if online && time+dt>scenario.maxDuration,terminated=true;break;end
            v=max(0,data(end,5)-motion.deceleration*dt);
            w=sign(data(end,6))*max(0,abs(data(end,6))-motion.yawAcceleration*dt);
            if online,observe([pose(1:2);path(end,:)]);superState="clear";end
            moveStep(dt,v,w,path(end,:),"docking");
        end
    end
    if rerouted,item.path=replacement;end
    end
    if terminated,break;end
    rotateTo(item.targetPose(3));
    if terminated,break;end
    errorAngle=atan2(sin(pose(3)-item.targetPose(3)),cos(pose(3)-item.targetPose(3)));
    assert(norm(pose(1:2)-item.targetPose(1:2))<=motion.positionTolerance && ...
        abs(errorAngle)<=motion.headingTolerance && all(abs(data(end,5:6))<1e-9), ...
        'Warehouse:DockingFailed','Docking error at %s: %.4f m, %.4f rad.',target,norm(pose(1:2)-item.targetPose(1:2)),errorAngle);
    record("arrived");
    station=map.docks(find([map.docks.id]==target,1));
    if station.kind=="loading"
        record("loading_started"); dwell(motion.loadingTime,"loading");
        if terminated || rerouted,break;end
        completed=completed+1;data(end,9)=completed; record("loading_completed");
    elseif station.kind=="unloading"
        assert(completed==numel(plan.rackIds),'Warehouse:IncompletePicks','All racks must be served before unloading.');
        record("unloading_started");dwell(motion.unloadingTime,"unloading");
        if terminated || rerouted,break;end
        record("unloading_completed");
    else
        record("returned");
    end
end
intervalPhases=phases;
if terminated,phases(end)="blocked";record("blocked");else,phases(end)="completed";end
clearance=zeros(size(data,1),1);
for k=1:size(data,1)
    clearance(k)=warehouse.geometry.footprintClearance(data(k,2:4),map,cfg);
end
% Rigid-body displacement bounds clearance between neighbouring samples.
travel=vecnorm(diff(data(:,2:3)),2,2)+cfg.robot.radius*abs(diff(data(:,4)));
bound=min(clearance(1:end-1),clearance(2:end))-travel/2;
assert(all(bound>=cfg.robot.margin),'Warehouse:Collision','The swept footprint must retain the design margin.');
trajectory=array2table([data clearance],'VariableNames', ...
    {'Time_s','X_m','Y_m','Heading_rad','Speed_mps','YawRate_radps','Distance_m','Leg','CompletedPicks','Clearance_m'});
trajectory.Phase=phases;trajectory.Target=targets;
trajectory.ReferenceX_m=references(:,1);trajectory.ReferenceY_m=references(:,2);
trajectory.TrackingError_m=vecnorm(data(:,2:3)-references,2,2);
log=struct2table(events);
loadTime=sum(diff(data(:,1)).*(intervalPhases(2:end)=="loading"));
unloadTime=sum(diff(data(:,1)).*(intervalPhases(2:end)=="unloading"));
rotationTime=sum(diff(data(:,1)).*(phases(2:end)=="rotating"));
run.status="completed";if terminated,run.status="blocked";end
run.plan=plan;run.map=map;run.config=cfg;run.motion=motion;
run.trajectory=trajectory;run.events=log;
run.metrics=table(time,plan.distance_m,distance,time-loadTime-unloadTime,rotationTime,loadTime,unloadTime, ...
    min(clearance),min(bound),completed,norm(pose(1:2)-map.docks(map.waitingIndex).pose(1:2)), ...
    'VariableNames',{'Duration_s','PlannedDistance_m','TravelledDistance_m','TravelAndTurningTime_s', ...
    'TurningTime_s','LoadingTime_s','UnloadingTime_s','MinimumSampledClearance_m', ...
    'ConservativeClearanceBound_m','CompletedPicks','FinalPositionError_m'});
tracking=trajectory.TrackingError_m(phases=="travelling" | phases=="docking");
if isempty(tracking),run.metrics.MaximumTrackingError_m=NaN;
else,run.metrics.MaximumTrackingError_m=max(tracking);end
run.metrics.RMSTrackingError_m=sqrt(mean(tracking.^2));
run.metrics.DistanceDifference_m=distance-plan.distance_m;
run.metrics.RotationEpisodes=sum(phases(2:end)=="rotating" & phases(1:end-1)~="rotating");
run.engine="closed_loop_stepwise";
run.replanning=replanning;run.routeUpdates=routeUpdates;
run.replans=struct2table(replanLog);
run.metrics.ReplanAttempts=numel(replanLog);
run.metrics.AcceptedReplans=numel(routeUpdates);
run.terminationReason="mission_completed";
if terminated,run.terminationReason="simulation_time_limit";end
run.metrics.Status=run.status;
run.metrics.TerminationReason=run.terminationReason;
if online
    if randomTraffic
        run.pedestrians=pedestrianLog;
        run.crowdSummary=table(string({scenario.people.id})',crowd.visits,crowd.distance,crowd.yieldTime,crowd.emergencyStops,crowd.redirections, ...
            'VariableNames',{'Pedestrian','DestinationsReached','Distance_m','YieldTime_s','EmergencyStops','Redirections'});
    end
    run=warehouse.safety.finaliseOnlineRun(run,scenario,superLog,superStates);
end
    function moveStep(dt,v,w,reference,phase)
        vm=(data(end,5)+v)/2;wm=(data(end,6)+w)/2;
        middle=pose(3)+wm*dt/2;
        shift=vm*dt;
        if abs(wm)>1e-10,shift=2*vm/wm*sin(wm*dt/2);end
        next=[pose(1:2)+shift*[cos(middle) sin(middle)] pose(3)+wm*dt];
        if phase=="travelling" && norm(next(1:2)-item.targetPose(1:2))<=motion.dockingDistance,phase="docking";end
        append(dt,next,v,w,phase,vm*dt,reference);
    end
    function append(dt,poses,v,w,phase,ds,reference)
        count=numel(dt);if count==0,return;end
        if online
            assert(count==1,'Warehouse:OnlineStep','Online execution must use scalar time steps.');
            assert(time+dt<=scenario.maxDuration,'Warehouse:PedestrianTimeout','Mission exceeded its time limit.');
            if randomTraffic
                velocity=v*[cos(poses(3)) sin(poses(3))];
                crowd=warehouse.safety.advanceCrowd(crowd,dt,poses,velocity,map,cfg,scenario);
                pedestrianLog(end+1,:)=reshape(crowd.position',1,[]);
            end
            superLog(end+1,:)=[time+dt scale double(conflict)];
            superStates(end+1,1)=superState;
        end
        if nargin<7,reference=poses(:,1:2);end
        references=[references;reference];
        rows=[time+dt poses v w distance+ds repmat(leg,count,1) repmat(completed,count,1)];
        data=[data;rows]; phases=[phases;repmat(phase,count,1)]; targets=[targets;repmat(target,count,1)];
        time=rows(end,1);pose=poses(end,:);distance=rows(end,7);
    end
    function rotateTo(heading)
        angle=atan2(sin(heading-pose(3)),cos(heading-pose(3)));
        p=warehouse.execution.motionProfile(abs(angle),motion.maxYawRate,motion.yawAcceleration,motion.yawAcceleration,motion.sampleTime);
        if isempty(p),return;end
        if online
            rr=[p(:,1) repmat(pose(1:2),size(p,1),1) pose(3)+sign(angle)*p(:,2) zeros(size(p,1),1) sign(angle)*p(:,3)];
            followReference(rr,"rotating");
            if terminated,return;end
            while max(abs(data(end,5:6)))>1e-10
                dt=motion.sampleTime;
                if online && time+dt>scenario.maxDuration,terminated=true;return;end
                v=max(0,data(end,5)-motion.deceleration*dt);
                w=sign(data(end,6))*max(0,abs(data(end,6))-motion.yawAcceleration*dt);
                observe(pose(1:2));superState="clear";
                moveStep(dt,v,w,pose(1:2),"rotating");
            end
            return
        end
        count=size(p,1)-1;
        poses=[repmat(pose(1:2),count,1) pose(3)+sign(angle)*p(2:end,2)];
        append(p(2:end,1),poses,zeros(count,1),sign(angle)*p(2:end,3),"rotating",zeros(count,1));
    end
    function dwell(duration,phase)
        if duration==0,return;end
        dt=unique([motion.sampleTime:motion.sampleTime:duration duration])';
        count=numel(dt);
        if online
            elapsed=0;
            for j=1:count
                step=dt(j)-elapsed;elapsed=dt(j);
                if online && time+step>scenario.maxDuration,terminated=true;return;end
                observe(pose(1:2));superState="service";
                append(step,pose,0,0,phase,0,pose(1:2));
            end
            return
        end
        append(dt,repmat(pose,count,1),zeros(count,1),zeros(count,1),phase,zeros(count,1));
    end
    function observe(future)
        if randomTraffic,people=crowd.position;velocity=crowd.velocity;
        else,[people,velocity]=warehouse.safety.pedestrianState(time,scenario);end
        visible=warehouse.safety.visiblePedestrians(pose,people,map,scenario.sensorRange);
        [conflict,zoneOccupied]=warehouse.safety.crossingConflict(future,people,velocity,visible,zoneOccupied,cfg,scenario);
        if conflict,clearSince=-inf;elseif ~isfinite(clearSince),clearSince=time;end
    end
    function followReference(ref,phase)
        clock=0;step=motion.sampleTime;
        values=griddedInterpolant(ref(:,1),ref(:,2:end),'linear');
        while clock<ref(end,1)-1e-10
            if online && time+step>scenario.maxDuration,terminated=true;return;end
            look=min(ref(end,1),clock+(0:0.3:5)');
            ahead=values(look);
            future=[pose(1:2);ahead(:,1:2)];
            if phase=="rotating" && ~isfield(scenario,'kind'),future=[future;futurePath];end
            observe(future);
            blocked=conflict || time-clearSince<scenario.clearTime;
            if blocked && max(abs(data(end,5:6)))<1e-9
                if isnan(holdSince),holdSince=time;end
            else
                holdSince=NaN;
            end
            if phase=="travelling" && replanning.enabled && ~isnan(holdSince) && ...
                    time-holdSince>=replanning.holdTime && time-lastAttempt>=replanning.retryInterval && ...
                    numel(replanLog)<replanning.maxAttempts
                lastAttempt=time;
                if randomTraffic,people=crowd.position;velocity=crowd.velocity;
                else,[people,velocity]=warehouse.safety.pedestrianState(time,scenario);end
                visible=warehouse.safety.visiblePedestrians(pose,people,map,scenario.sensorRange);
                dock=map.docks(find([map.docks.id]==target,1));
                [replacement,reason]=warehouse.planning.avoidPeople(pose,dock,map,cfg,people,velocity,visible,scenario,replanning);
                lengthNew=sum(vecnorm(diff(replacement),2,2));
                replanLog(end+1)=struct('Time_s',time,'Target',target,'Outcome',reason,'Length_m',lengthNew);
                if ~isempty(replacement)
                    record("route_replanned");
                    routeUpdates(end+1)=struct('time_s',time,'leg',leg,'path',replacement);
                    rerouted=true;scale=0;holdSince=NaN;return
                end
            end
            desiredScale=double(~blocked);
            if blocked && max(abs(data(end,5:6)))<1e-10,scale=0;end
            nextScale=max(0,min(1,scale+max(-scenario.scaleRate*step,min(scenario.scaleRate*step,desiredScale-scale))));
            nextClock=min(ref(end,1),clock+step*(scale+nextScale)/2);
            a=values(clock);b=values(nextClock);
            delta=a(1:2)-pose(1:2);
            ex=cos(pose(3))*delta(1)+sin(pose(3))*delta(2);
            ey=-sin(pose(3))*delta(1)+cos(pose(3))*delta(2);
            et=atan2(sin(a(3)-pose(3)),cos(a(3)-pose(3)));
            gains=motion.trackingGains;
            v=nextScale*(b(4)*cos(et)+gains(1)*ex);
            w=nextScale*(b(5)+gains(2)*b(4)*ey+gains(3)*sin(et));
            v=max(0,min(motion.maxSpeed,v));
            v=max(data(end,5)-motion.deceleration*step,min(data(end,5)+motion.acceleration*step,v));
            w=max(-motion.maxYawRate,min(motion.maxYawRate,w));
            w=max(data(end,6)-motion.yawAcceleration*step,min(data(end,6)+motion.yawAcceleration*step,w));
            scale=nextScale;clock=nextClock;
            superState="clear";
            if blocked,superState="slowing";elseif scale<1-1e-8,superState="resuming";end
            if scale==0 && max(abs([v w]))<1e-9,superState="hold";end
            moveStep(step,v,w,b(1:2),phase);
        end
    end
    function record(type)
        events(end+1)=struct('type',type,'station',target,'time_s',time, ...
            'x_m',pose(1),'y_m',pose(2),'heading_rad',pose(3),'completedPicks',completed);
    end
end
