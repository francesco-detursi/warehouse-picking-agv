function exportPedestrianHold(run,folder)
q=run.trajectory;indices=find(q.SupervisorState=="hold");
if isempty(indices),return;end
[~,j]=min(q.PedestrianClearance_m(indices));index=indices(j);
f=figure('Visible','off','Color','white','Position',[40 40 1240 760]);cleanup=onCleanup(@()close(f));
warehouse.view.drawLayout(f,run.map,run.config,false);ax=findobj(f,'Tag','WarehouseMap');
delete(findobj(ax,'Type','patch'));
lines=findobj(ax,'Type','line');for k=1:numel(lines),if isequal(lines(k).Color,[1 1 1]),delete(lines(k));end,end
labels=findall(f,'Type','textboxshape');
for k=1:numel(labels)
 if labels(k).Position(1)>=0.75,delete(labels(k));
 elseif contains(string(labels(k).String),'Single-robot layout'),labels(k).String='Warehouse A  /  Pedestrian hold';end
end
plot(ax,q.X_m(1:index),q.Y_m(1:index),'Color',[0.04 0.35 0.66],'LineWidth',1.5);
pose=q{index,{'X_m','Y_m','Heading_rad'}};body=warehouse.geometry.robotPolygon(pose,run.config);
patch(ax,body(:,1),body(:,2),[0.04 0.35 0.66],'EdgeColor',[0.12 0.18 0.23],'LineWidth',1.2);
plot(ax,[pose(1) pose(1)+0.38*cos(pose(3))],[pose(2) pose(2)+0.38*sin(pose(3))],'w-','LineWidth',2);
warehouse.view.drawPedestrians(ax,run.scenario,reshape(run.pedestrians(index,:),2,[])');
lines={'PEDESTRIAN HOLD',sprintf('Time: %.1f s',q.Time_s(index)),'Robot speed: 0.00 m/s', ...
 sprintf('Pedestrian gap: %.2f m',q.PedestrianClearance_m(index)),'Orange circles: pedestrians','Restart requires a clear corridor',['Mission status: ' char(run.status)],'Station order remains unchanged'};
if isfield(run,'crowdSummary')
 lines{6}=sprintf('Traffic seed: %.0f',run.scenario.seed);
 lines{7}='Random destinations and pauses';
 lines{8}=['Mission status: ' char(run.status)];
end
for k=1:numel(lines)
 annotation(f,'textbox',[0.76 0.8-(k-1)*0.075 0.235 0.045],'String',lines{k}, ...
 'FontName','Arial','FontSize',11,'EdgeColor','none','Interpreter','none');
end
exportgraphics(f,fullfile(folder,'pedestrian_hold.png'),'Resolution',250);
exportgraphics(f,fullfile(folder,'pedestrian_hold.pdf'),'ContentType','vector');
end
