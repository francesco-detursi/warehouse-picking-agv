function [conflict,zoneOccupied]=crossingConflict(future,people,velocity,visible,zoneOccupied,cfg,scenario)
conflict=false;
 occupied=false(size(visible));
 for person=1:size(people,1)
  for zone=1:size(scenario.crossingZones,1)
   box=scenario.crossingZones(zone,:);
   delta=max(max(box(1:2)-people(person,:),0),people(person,:)-box(1:2)-box(3:4));
   occupied(person)=occupied(person) || norm(delta)<=scenario.radius;
  end
 end
 if any(visible & occupied),zoneOccupied=true;
 elseif all(visible) && ~any(occupied),zoneOccupied=false;end
 if zoneOccupied
  for zone=1:size(scenario.crossingZones,1)
   box=scenario.crossingZones(zone,:);
   delta=max(max(box(1:2)-future,0),future-box(1:2)-box(3:4));
   if any(vecnorm(delta,2,2)<cfg.robot.radius+scenario.separation+0.15),conflict=true;end
  end
 end
 for person=find(visible)'
  predicted=people(person,:)+(0:0.2:scenario.predictionTime)'.*velocity(person,:);
  for predictionIndex=1:size(predicted,1)
   if any(vecnorm(future-predicted(predictionIndex,:),2,2)<cfg.robot.radius+scenario.radius+scenario.separation+0.15)
    conflict=true;break
   end
  end
 end
end
