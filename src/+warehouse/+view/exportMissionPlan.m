function exportMissionPlan(plan,map,cfg,folder)
fig=figure('Visible','off','Color','white','Position',[40 40 1240 760]);
cleanup=onCleanup(@()close(fig));
warehouse.view.drawLayout(fig,map,cfg,false);
labels=findall(fig,'Type','textboxshape');
for k=1:numel(labels)
 position=labels(k).Position;
 if position(1)>=0.75,delete(labels(k));
 elseif contains(string(labels(k).String),'Single-robot layout')
  labels(k).String='Warehouse A  /  Planned collection mission';
 elseif contains(string(labels(k).String),'Dashed boxes:')
  labels(k).String='Blue: collection / unloading  |  Green dashed: return to W  |  Arrows: travel direction';
 end
end
ax=findobj(fig,'Tag','WarehouseMap');
warehouse.view.drawMissionRoute(ax,plan);
for k=1:numel(plan.rackIndices)
 rect=map.obstacles(plan.rackIndices(k),:);
 rectangle(ax,'Position',rect,'EdgeColor',[0.04 0.35 0.66],'LineWidth',2);
 text(ax,rect(1)+rect(3)/2,rect(2)+rect(4)+0.35,string(k),'HorizontalAlignment','center', ...
  'VerticalAlignment','bottom','Color',[0.04 0.35 0.66],'FontWeight','bold','FontSize',11, ...
  'BackgroundColor','white','Margin',1);
end
line('MISSION PLAN',0.80,13,true);
mode='Follow selected order';if plan.request.orderMode=="optimised",mode='Optimise visit order';end
line(mode,0.74,11,false);
line('PLANNED DISTANCE',0.68,10,false);
line(sprintf('%.2f m',plan.distance_m),0.635,16,true);
line('VISIT SEQUENCE',0.59,12,true);
line('W - Start',0.55,10,false);
for k=1:numel(plan.rackIds),line(sprintf('%d.   %s',k,plan.rackIds(k)),0.55-k*0.038,10,false);end
line('D - Unload',0.55-(numel(plan.rackIds)+1)*0.038,10,false);
line('W - Return',0.55-(numel(plan.rackIds)+2)*0.038,10,false);
line('P01-P06: loading docks',0.13,9,false);
line('D: unloading  |  W: waiting',0.095,9,false);
exportgraphics(fig,fullfile(folder,'mission_plan.png'),'Resolution',300);
exportgraphics(fig,fullfile(folder,'mission_plan.pdf'),'ContentType','vector');
 function line(value,y,size,bold)
  weight='normal';if bold,weight='bold';end
  annotation(fig,'textbox',[0.76 y 0.235 0.04],'String',value,'EdgeColor','none', ...
   'FontName','Arial','FontSize',size,'FontWeight',weight,'Interpreter','none','Color',[0.12 0.18 0.23]);
 end
end
