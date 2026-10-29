function visible=visiblePedestrians(pose,positions,map,range)
visible=vecnorm(positions-pose(1:2),2,2)<=range;
for k=find(visible)'
 a=pose(1:2);b=positions(k,:);direction=b-a;
 for j=1:size(map.obstacles,1)
  box=map.obstacles(j,:);low=0;high=1;
  for axis=1:2
   if abs(direction(axis))<1e-12
    if a(axis)<box(axis) || a(axis)>box(axis)+box(axis+2),low=2;break;end
   else
    limits=sort(([box(axis) box(axis)+box(axis+2)]-a(axis))/direction(axis));
    low=max(low,limits(1));high=min(high,limits(2));
   end
  end
  if low<=high && high>=0 && low<=1,visible(k)=false;break;end
 end
end
end
