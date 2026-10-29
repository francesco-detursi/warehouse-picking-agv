function drawMissionRoute(ax,plan)
for k=1:numel(plan.legs)
 p=plan.legs(k).path; color=[0.04 0.35 0.66]; style='-';
 if k==numel(plan.legs),color=[0 0.43 0.35];style='--';end
 plot(ax,p(:,1),p(:,2),style,'Color',color,'LineWidth',1.5, ...
  'Tag','PlannedRoute','HitTest','off','PickableParts','none');
 lengths=vecnorm(diff(p),2,2);[~,j]=max(lengths);
 if lengths(j)>0.5
  direction=(p(j+1,:)-p(j,:))/lengths(j);
  centre=(p(j+1,:)+p(j,:))/2;
  quiver(ax,centre(1)-0.2*direction(1),centre(2)-0.2*direction(2), ...
   0.4*direction(1),0.4*direction(2),0,'Color',color,'LineWidth',1.2, ...
   'MaxHeadSize',1.5,'Tag','PlannedRoute','HitTest','off','PickableParts','none');
 end
end
labels=findobj(ax,'Type','text');
for k=1:numel(labels)
 if startsWith(string(labels(k).String),"P0")
  labels(k).BackgroundColor=[0.965 0.975 0.982]; labels(k).Margin=1;
 end
end
uistack(labels,'top');
end
