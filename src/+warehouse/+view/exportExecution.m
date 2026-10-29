function exportExecution(run,folder)
fig=figure('Visible','off','Color','white','Position',[40 40 1240 760]);cleanup=onCleanup(@()close(fig));
warehouse.view.drawLayout(fig,run.map,run.config,false);
labels=findall(fig,'Type','textboxshape');
for k=1:numel(labels)
    if labels(k).Position(1)>=0.75,delete(labels(k));
    elseif contains(string(labels(k).String),'Single-robot layout'),labels(k).String=['Warehouse A  /  Mission ' char(run.status)];
        if isfield(run,'crowdSummary'),labels(k).String=sprintf('Warehouse A  /  %d pedestrians  /  Seed %.0f  /  %s',numel(run.scenario.people),run.scenario.seed,run.status);end
    elseif contains(string(labels(k).String),'Dashed boxes:'),labels(k).String='Blue: travelled path  |  Kinematic tracking model  |  Loading / unloading represented by stationary service times';end
end
ax=findobj(fig,'Tag','WarehouseMap');
delete(findobj(ax,'Type','patch'));
handles=findobj(ax,'Type','line');
for j=1:numel(handles),if isequal(handles(j).Color,[1 1 1]),delete(handles(j));end,end
plot(ax,run.trajectory.X_m,run.trajectory.Y_m,'Color',[0.04 0.35 0.66],'LineWidth',1.5);
for j=run.plan.rackIndices(1:run.metrics.CompletedPicks)
    rectangle(ax,'Position',run.map.obstacles(j,:),'EdgeColor',[0 0.43 0.35],'LineWidth',2);
end
warehouse.view.protectMapLabels(ax);
pose=run.trajectory{end,{'X_m','Y_m','Heading_rad'}};
body=warehouse.geometry.robotPolygon(pose,run.config);
patch(ax,body(:,1),body(:,2),[0.04 0.35 0.66],'EdgeColor',[0.12 0.18 0.23],'LineWidth',1.2);
plot(ax,[pose(1) pose(1)+0.38*cos(pose(3))],[pose(2) pose(2)+0.38*sin(pose(3))],'w-','LineWidth',2);
if isfield(run,'scenario')
 warehouse.view.drawPedestrians(ax,run.scenario,reshape(run.pedestrians(end,:),2,[])');
end
m=run.metrics;
lines={char(upper('Mission '+run.status)),sprintf('%d / %d racks served',m.CompletedPicks,numel(run.plan.rackIds)), ...
    sprintf('Distance: %.2f m',m.TravelledDistance_m),sprintf('Total time: %.1f s',m.Duration_s), ...
    sprintf('Travel / turns: %.1f s',m.TravelAndTurningTime_s),sprintf('Loading: %.1f s',m.LoadingTime_s), ...
    sprintf('Unloading: %.1f s',m.UnloadingTime_s),['Last target: ' char(run.trajectory.Target(end))]};
if isfield(run,'scenario')
 lines=[lines,{sprintf('Pedestrian hold: %.1f s',m.PedestrianHoldTime_s),sprintf('Pedestrian stops: %d',m.PedestrianHoldEpisodes)}];
end
if isfield(run,'replans'),lines=[lines,{sprintf('Route updates: %d',m.AcceptedReplans)}];end
for k=1:numel(lines)
    weight='normal';if k==1,weight='bold';end
    annotation(fig,'textbox',[0.76 0.82-(k-1)*0.065 0.235 0.045],'String',lines{k}, ...
        'FontName','Arial','FontSize',11,'FontWeight',weight,'EdgeColor','none','Interpreter','none');
end
exportgraphics(fig,fullfile(folder,char('mission_'+run.status+'.png')),'Resolution',300);
exportgraphics(fig,fullfile(folder,char('mission_'+run.status+'.pdf')),'ContentType','vector');
warehouse.view.exportMotion(run,folder);
if isfield(run,'scenario'),warehouse.view.exportPedestrianHold(run,folder);end
end
